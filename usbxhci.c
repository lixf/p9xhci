/**
 * USB eXtensible Host Controller Interface driver.
 *
 **/

/**
 * Bug list
 * 1. Support multiple controllers **DONE**
 * 2. Command Ring operation for multiple in-flight commands
 *      2.1 memcpy in plan 9
 * 3. What's the job of init() vs. reset()
 * 4. Should I use qlock in portreset()?
 * 5. The format for portstatus. 
 * 6. 64b compatibility?
 * 7. Can remove the need for a event ring deq in ctlr. instead we can use 
 * the structure in Sw_ring. 
 * 8. Need a clean way to report critical errors (instead of panicking)
 *
 *
 * TODO List
 * 1. Command completion report slot ID
 * 2. Try diff sizes for event seg table
 * 3. Try diff sizes for slot enabled
 **/

#include"u.h"
#include"../port/lib.h"
#include"mem.h"
#include"dat.h"
#include"fns.h"
#include"io.h"
#include"../port/error.h"
#include"usb.h"

#define XHCI_DEBUG

#ifdef XHCI_DEBUG
#define __ddprint print
#else 
#define __ddprint if(0) print
#endif

/* static values -- read from CAPREG */
int debug = 1;

/* Changable hardcoded values */
#define XHCI_MAXSLOTSEN 2   /* Defines the max. num of slots enabled by sw */
#define CMD_RING_SIZE 32    /* Defines the number of TRBs in a CMD ring */
#define EVENT_SEGTABLE 1    /* Defines the max. num of seg table entries */

#define _64B 64
#define _4KB (4 << 10)

/* Hardware Constants */
#define CAPLENGTH_OFF (0x0)
#define HCSPARAMS1_OFF (0x4)
#define DB_OFF (0x14)
#define RTS_OFF (0x18)              
#define PORTSC_ENUM_OFF (0x10)

/* declarations */
struct Ctlr;  
struct Trb;
struct Td;
struct SlotCtx;
struct EpCtx;
struct Packed32B;   
struct Packed16B;   
struct EventSegTabEntry;

/* Internal definitions */
struct Sw_ring 
{
    /* defines the fields for a TRB ring used internally */
    uint phys;  
    uint virt;
    uint curr;
    char cycle; 
    uint length; 
};

struct Oper {
    uint base; 
    uint config; 
    uint usbsts; 
    uint dcbaap_lo;
    uint dcbaap_hi;
    uint crcr_lo;
    uint crcr_hi;
    uint usbcmd; 
    uint portsc; 
};

struct Runt {
    uint base; 
    uint iman; 
    uint erstsz; 
    uint erstba_lo;
    uint erstba_hi;
    uint erdp_lo;
    uint erdp_hi;
};

/* Definitions */
struct Ctlr 
{
    Lock; 
    QLock portlck; 
    Pcidev *pcidev; 
    int active; 

    /* Hardware values */ 
    void *xhci;
    struct Oper oper;
    struct Runt runt;
    uint caplength; 
    uint num_port; 
    uint db_off; 
    uint max_slot;

    /* software values */
    uint devctx_bar; 
    struct Sw_ring cmd_ring;  
    struct Sw_ring event_ring;
    struct Sw_ring event_segtable;
};

struct Trb 
{
	volatile uvlong qwTrb0;
	volatile uint dwTrb2;
    volatile uint dwTrb3;
#define TYPE_GET(x)      (((x) >> 10) & 0x3F)
#define TYPE_SET(x)      (((x) & 0x3F) << 10)
#define CYCLE_BIT        (1U << 0)
#define EP_GET(x)        (((x) >> 16) & 0x1F)
#define EP_SET(x)        (((x) & 0x1F) << 16)
#define SLOT_GET(x)      (((x) >> 24) & 0xFF)
#define SLOT_SET(x)      (((x) & 0xFF) << 24)

/* Commands */
#define TYPE_ENABLE_SLOT   0x09

/* Events */
#define EVENT_CMD_COMPLETE 0x21
#define EVENT_PORT_STS_CHANGE  0x22

};

struct Td 
{
    volatile int id; 
    volatile struct Trb *trbs;
    volatile int length;
}; 

struct SlotCtx 
{
    /* word 0 */
    uint routeStr;  
    char speed;     
    char mmt;       
    char hub;       
    char ctxEntry;  
    
    /* word 1 */
    uint maxExit;   
    char rootPort;  
    char numPort;   

    /* word 2 */
    char tthubSlot;  
    char ttportNum;  
    char ttt;        
    uint intrTarget; 

    /* word 3 */
    char devAddr;
    char slotState;
}; 

struct EpCtx 
{
    /* word 0 */
    char epstate;   
    char mult;      
    char maxPStream;
    char lsa;       
    char interval;  
    char maxEsitHi; 
    
    /* word 1 */
    char cErr;       
    char epType;     
    char hostInitDis;
    char maxBurst;   
    uint maxPktSize; 

    /* word 2 */
    char deqCyState;
    uint trDeqPtrLo;

    /* word 3 */
    uint trDeqPtrHi;

    /* word 4 */
    uint aveTrbLen; 
    char maxEsitLo; 
}; 

struct EventSegTabEntry 
{
    uint ringSegBarLo;
    uint ringSegBarHi; 
    short rsvd1; 
    unsigned short ringSegSize;
    uint rsvd2; 
}; 

struct Packed32B 
{
    volatile uint words[8]; 
};

struct Packed16B
{
    volatile uint words[4]; 
};

typedef struct Ctlr Ctlr; 
typedef struct Trb Trb; 
typedef struct Td Td; 
typedef struct SlotCtx slotCtx; 
typedef struct EpCtx epCtx; 
typedef struct Packed32B packed32B; 
typedef struct Packed16B packed16B; 
typedef struct EventSegTabEntry eventSegTabEntry; 

// Local debug function defs
static void _dump_trb(Trb *t);
static void _dump_cmd_ring(struct Sw_ring *ring);
static void _dump_event_ring(struct Sw_ring *ring);
static void _dump_event_segtable(struct Sw_ring *ring); 
static void dump(Hci *hp); 


/*********************************************
 * xHCI specific functions
 *********************************************/
static void
setup_oper(Ctlr *ctlr, uint base)
{
    struct Oper *oper = &(ctlr->oper); 
    oper->base = base; 
    oper->usbcmd = base; 
    oper->usbsts = base + 0x04; 
    oper->config = base + 0x38; 
    oper->dcbaap_lo = base + 0x30; 
    oper->dcbaap_hi = base + 0x34; 
    oper->crcr_lo   = base + 0x18; 
    oper->crcr_hi   = base + 0x1C; 
    oper->portsc    = base + 0x400; 
}

static void
setup_runt(Ctlr *ctlr, uint base)
{
    struct Runt *runt = &(ctlr->runt); 
    runt->base = base; 
    runt->iman = base + 0x20; 
    runt->erstsz    = base + 0x28; 
    runt->erstba_lo = base + 0x30; 
    runt->erstba_hi = base + 0x34; 
    runt->erdp_lo   = base + 0x38; 
    runt->erdp_hi   = base + 0x3C; 
}

/** @brief These functions parse out the context structures and pack the data
 *  into a more compact 32B struct for hardware use
 *  
 *  @param[in] slot The input slot context
 *  @param[in] ep The input endpoint context
 *  @param[out] packed The 32B packed representation of the contexts
 *  
 *  @fault never
 *  @return void
 **/
static inline void
pack_slot_ctx(packed32B *packed, slotCtx *slots) 
{
    uint word0 = (slots->ctxEntry & 0x1F) << 27; 
    word0 = word0 | ((slots->hub & 0x1) << 26);
    word0 = word0 | ((slots->mmt & 0x1) << 25);
    word0 = word0 | ((slots->speed & 0xF) << 20);
    word0 = word0 | (slots->routeStr & 0xFFFFF); 

    uint word1 = (slots->numPort & 0xFF) << 24; 
    word1 = word1 | ((slots->rootPort & 0xFF) << 26);
    word1 = word1 | (slots->maxExit & 0xFFFF);
  
    uint word2 = (slots->intrTarget & 0x3FF) << 22; 
    word2 = word2 | ((slots->ttt & 0x3) << 16);
    word2 = word2 | ((slots->ttportNum & 0xFF) << 8);
    word2 = word2 | (slots->tthubSlot & 0xFF);
    
    uint word3 = (slots->slotState & 0x1F) << 27;
    word3 = word3 | (slots->devAddr & 0xFF);
    
    packed->words[0] = word0;
    packed->words[1] = word1;
    packed->words[2] = word2;
    packed->words[3] = word3;
    return; 
}

static inline void
unpack_slot_ctx(packed32B *packed, slotCtx *slots) 
{
    return; 
}

static inline void
pack_ep_ctx(packed32B *packed, epCtx *ep) 
{
    uint word0 = (ep->maxEsitHi & 0xFF) << 24; 
    word0 = word0 | ((ep->interval & 0xFF) << 16);
    word0 = word0 | ((ep->lsa & 0x1) << 15);
    word0 = word0 | ((ep->maxPStream & 0x1F) << 10);
    word0 = word0 | ((ep->mult & 0x3) << 8);
    word0 = word0 | (ep-> epstate & 0x7); 

    uint word1 = (ep->maxPktSize & 0xFFFF) << 16; 
    word1 = word1 | ((ep->maxBurst & 0xFF) << 8);
    word1 = word1 | ((ep->hostInitDis & 0x1) << 7);
    word1 = word1 | ((ep->epType & 0x3) << 3);
    word1 = word1 | ((ep->cErr & 0x3) << 1);
  
    uint word2 = (ep->trDeqPtrLo & 0xFFFFFFF0) << 4; 
    word2 = word2 | (ep->deqCyState & 0x1);
    
    uint word3 = ep->trDeqPtrHi; 
    
    uint word4 = (ep->maxEsitLo & 0xFF) << 24;
    word4 = word4 | (ep->aveTrbLen & 0xFFFF);
    
    packed->words[0] = word0;
    packed->words[1] = word1;
    packed->words[2] = word2;
    packed->words[3] = word3;
    packed->words[4] = word4;
    return; 
}

static inline void
unpack_ep_ctx(packed32B *packed, epCtx *ep) 
{
    return; 
}

/** @brief These functions define the default setups for a slot/ep context 
 *  
 *  @param[in/out] slot_ctx The slot context to configure
 *  @param[in/out] ep_ctx   The ep context to configure
 *  
 *  @fault never
 *  @return void
 **/
static inline void 
setup_default_slot_ctx(slotCtx *slot_ctx) 
{
    // now configure the slot and endpoint; then pack into 32B structs
    // TODO don't write all zeros
    slot_ctx->routeStr  = 0; 
    slot_ctx->speed     = 0; 
    slot_ctx->mmt       = 0; 
    slot_ctx->hub       = 0; 
    slot_ctx->ctxEntry  = 0; 

    slot_ctx->maxExit   = 0; 
    slot_ctx->rootPort  = 0; 
    slot_ctx->numPort   = 0; 
    
    slot_ctx->intrTarget = 0; 
    
    slot_ctx->devAddr   = 0; 
    slot_ctx->slotState = 0; 
    return;
}

static inline void
setup_default_ep_ctx(epCtx *ep_ctx) 
{
    // TODO write something 
    ep_ctx->epstate       = 0;
    ep_ctx->hostInitDis   = 0;
    ep_ctx->trDeqPtrLo    = 0;
    ep_ctx->trDeqPtrHi    = 0;
    return; 
}

/** @brief This function will write to one of the registers
 *  NOTE: the offset is based off of the MMIO PCI BAR!
 **/
static void
xhcireg_wr(Ctlr *ctlr, uint offset, uint mask, uint new) 
{
    uint addr = ((uint) ctlr->xhci) + offset; 
    uint read = *((uint *) addr);
    *((uint *) addr) = ((read & ~mask) | new);
}

static uint
xhcireg_rd(Ctlr *ctlr, uint offset, uint mask) 
{
    uint addr = ((uint) ctlr->xhci) + offset; 
    return (*((uint *) addr) & mask); 
}

static Ctlr* ctlrs[Nhcis];


static void inline
ring_bell(Ctlr *ctlr, uint index, uint db) 
{
#ifdef XHCI_DEBUG
    __ddprint("ringing doorbell index %d, val %#ux\n", index, db);
#endif
    assert(index <= ctlr->max_slot);
    xhcireg_wr(ctlr, (ctlr->db_off + index * sizeof(uint)), 0xFFFFFFFF, db);
}


static void inline 
send_command(Ctlr *ctlr, Trb *trb) 
{
    
    /* FIXME Bug #2 
     * What if there's some unfinished commands before the new command?
     * ctlr should probably have a index pointer to the next free slot 
     */
    if(ctlr == nil || ctlr->cmd_ring.virt == 0 || trb == nil){
        panic("xhci send command internal error\n");
    } else {

#ifdef XHCI_DEBUG
        __ddprint("sending command:\n");
        _dump_trb(trb);
#endif

        // FIXME Bug 2.1 Should use memcpy here
        Trb *dst = (Trb *)ctlr->cmd_ring.curr;
        Trb *src = (Trb *)trb;
        dst->qwTrb0 = src->qwTrb0;
        dst->dwTrb2 = src->dwTrb2;
        dst->dwTrb3 = src->dwTrb3;

        /* FIXME Assume it does not wrap around for now */
        ctlr->cmd_ring.curr += sizeof(struct Trb); 
    }
    return; 
}




/**********************************************
 * Top level interface functions -- debugging * 
 **********************************************/
static char*
seprintep(char *s, char *e, Ep *ep)
{
    return nil; 
}

static void
_dump_trb(Trb *t) 
{
    __ddprint("dumping TRB: \n");
    assert(t != nil); 
    __ddprint("qwTrb0 (data ptr low): %#ux\n", (uint)(t->qwTrb0 & 0xFFFFFFFF));
    __ddprint("qwTrb0 (data ptr high): %#ux\n", (uint)(t->qwTrb0 >> 32));
    __ddprint("dwTrb2 (status): %#ux\n", t->dwTrb2);
    __ddprint("dwTrb3 (status): %#ux\n", t->dwTrb3);
    __ddprint("cycle bit: %d\n", (t->dwTrb3 & CYCLE_BIT));
    return; 
}


static void
_dump_cmd_ring(struct Sw_ring *ring) 
{
    Trb *current; 
    __ddprint("***debug dump of command ring***\n");
    __ddprint("phys: %#ux, virt: %#ux, curr: %#ux, length: %#ux\n", 
        ring->phys, ring->virt, ring->curr, ring->length);
    for(uint i = 0; i < ring->length; i++){
        current = (Trb *)(ring->virt + i * sizeof(struct Trb)); 
        _dump_trb(current);  
    }
}

static void
_dump_event_ring(struct Sw_ring *ring) 
{
    Trb *current; 
    __ddprint("***debug dump of event ring***\n");
    __ddprint("phys: %#ux, virt: %#ux, curr: %#ux, length: %#ux\n", 
        ring->phys, ring->virt, ring->curr, ring->length);
    for(uint i = 0; i < ring->length; i++){
        current = (Trb *)(ring->virt + i * sizeof(struct Trb)); 
        _dump_trb(current);  
    }
}

static void
_dump_event_segtable(struct Sw_ring *ring) 
{
    eventSegTabEntry *current; 
    __ddprint("***debug dump of event segment table***\n");
    __ddprint("phys: %#ux, virt: %#ux, curr: %#ux, length: %#ux\n", 
        ring->phys, ring->virt, ring->curr, ring->length);
    for(uint i = 0; i < ring->length; i++){
        current = (eventSegTabEntry *)(ring->virt + i * sizeof(struct EventSegTabEntry)); 
        __ddprint("entry %d:\n", i);
        __ddprint("barLo: %#ux, size: %d\n", 
            current->ringSegBarLo, current->ringSegSize);
    }
}


static void
dump(Hci *hp) 
{
    Ctlr *ctlr;    
    ctlr = hp->aux;
    __ddprint("***debug dump of ctlr sw state***\n");
    __ddprint("active: %d, xhci: %#ux, oper: %#ux, runt: %#ux\n", 
        ctlr->active, (uint)ctlr->xhci, ctlr->oper.base, ctlr->runt.base);
    __ddprint("caplength: %#ux, num_port: %#ux, db_off: %#ux\n", 
        ctlr->caplength, ctlr->num_port, ctlr->db_off);
    __ddprint("max_slot: %#ux, devctx_bar: %#ux\n", 
        ctlr->max_slot, ctlr->devctx_bar);

    _dump_cmd_ring(&(ctlr->cmd_ring)); 
    _dump_event_ring(&(ctlr->event_ring));
    _dump_event_segtable(&(ctlr->event_segtable));
}


/**************************************************
 * Top level interface functions -- functionality * 
 **************************************************/
/* FIXME Bug #3 How is this function called? */
static void
init(Hci *hp)
{
    __ddprint("xhci init\n");
    return; 
}


static int
portreset(Hci *hp, int port, int on)
{
    __ddprint("xhci portreset\n");
    Ctlr *ctlr;

    /* FIXME Bug #4
     * Should I use the qlock as in UHCI driver?
     */
    
    if(on == 0){
        return 0;
    }
    
    ctlr = hp->aux;
    uint port_offset = ctlr->oper.portsc + (port-1) * PORTSC_ENUM_OFF; 
    xhcireg_wr(ctlr, port_offset, 0x10, 0x10); 
    int wait = 0;
    while(xhcireg_rd(ctlr, port_offset, 0x10)){
        delay(10);
        wait++; 
        if(wait == 100){
            __ddprint("xhci port %d reset timeout", port);
            return -1;
        }
    }

    __ddprint("port reset successful\n");
    return 0;
}
  
/** The USB3.0 spec does not allow/require software to enable a port, unlike
 *  the UHCI/OHCI spec. Only the XHC can enable a port upon a attachment event.
 *  However, we still allow disabling of a port through this function. 
 **/
static int
portenable(Hci *hp, int port, int on)
{ 
	Ctlr *ctlr;
	ctlr = hp->aux;
    qlock(&ctlr->portlck);

    __ddprint("xhci portenable\n");
    if(on == 1){
        __ddprint("xhci cannot allow enabling of a port\n");
        qunlock(&ctlr->portlck);
        return -1;
    } else {
        uint port_offset = ctlr->oper.portsc + (port-1) * PORTSC_ENUM_OFF; 
        uint port_sts = xhcireg_rd(ctlr, port_offset, 0x2);
        if(port_sts == 0){
            qunlock(&ctlr->portlck);
            return 0; 
        } else {
            xhcireg_wr(ctlr, port_offset, 0x2, 0x2);
            delay(5); 
            if(xhcireg_rd(ctlr, port_offset, 0x2)){
                __ddprint("port still enabled\n");
                qunlock(&ctlr->portlck);
                return -1;
            }
        }
    }
    qunlock(&ctlr->portlck);
    return 0; 
}

static int
portstatus(Hci *hp, int port)
{
	/* FIXME Bug #5
	 * We must return status bits as get port status hub request would do.
	 * What's the format?
     */
	Ctlr *ctlr;
	ctlr = hp->aux;
    
    uint port_offset = ctlr->oper.portsc + (port-1) * PORTSC_ENUM_OFF; 
    uint port_sts = xhcireg_rd(ctlr, port_offset, 0xFFFFFFFF);
    __ddprint("xhci portstatus %#ux for port num %d\n", port_sts, port);
    return -1; 
}

static void
epclose(Ep *ep)
{
    __ddprint("xhci epclose\n");
    return; 
}

static void
epopen(Ep *ep)
{
    __ddprint("xhci epopen\n");
    return; 
}

static long
epwrite(Ep *ep, void *a, long count)
{
    __ddprint("xhci epwrite\n");
    return -1;
}

static long
epread(Ep *ep, void *a, long count)
{
    __ddprint("xhci epread\n");
    return -1;
}


#define PORTSTS_PLS 0x1E0
#define PLS_POLLING 0x7
#define PLS_RXDETECT 0x5
static void 
handleattach(Hci *hp, Trb *psce) 
{
    Ctlr *ctlr;
    uint port_sts; 
    
    ctlr = hp->aux;
    uint port_id = (psce->qwTrb0 >> 24) & 0xFF;
    __ddprint("port id %d caused attachment event\n", port_id);

    uint port_offset = ctlr->oper.portsc + (port_id-1) * PORTSC_ENUM_OFF; 
    
    port_sts = xhcireg_rd(ctlr, port_offset, 0xFFFFFFFF); 
    __ddprint("port status %#ux\n", port_sts);

    uint port_state = xhcireg_rd(ctlr, port_offset, PORTSTS_PLS) >> 5;  
    if(port_state == PLS_POLLING){
        /* this is a USB2 device */
        portreset(hp, port_id, 1);
        
        port_sts = xhcireg_rd(ctlr, port_offset, 0xFFFFFFFF); 
        __ddprint("port status %#ux\n", port_sts);
        return; 
    }

    /* Now issue an Enable Slot Command */
    struct Trb slot_cmd; 
    slot_cmd.qwTrb0 = 0;
    slot_cmd.dwTrb2 = 0; 
    uint trb3 = TYPE_SET(TYPE_ENABLE_SLOT);
    trb3 |= EP_SET(0);
    trb3 |= ctlr->cmd_ring.cycle;
    slot_cmd.dwTrb3 = trb3; 
    
    send_command(ctlr, &slot_cmd);    
    
    /* Now ring the door bell for the XHC */
    uint db = 0;
    ring_bell(ctlr, 0, db); 
    
    return; 
}

static void
handlecmd(Hci *hp, Trb *trb) 
{
    Ctlr *ctlr;
    ctlr = hp->aux;
    
    uint slot_id = trb->dwTrb3 >> 24 & 0xFF; 
    __ddprint("USB slot assignment command returned slot ID: %#ux\n", slot_id);
    
    // potentially need to return the slot ID to map it with the physical port 
    // TODO #1
    return; 
}


/* The xHCI interrupt handler will print out the interrupt status and 
 * check the event ring
 */
static void
interrupt(Ureg*, void *arg)
{
    __ddprint("xhci interrupt\n");
	Hci *hp;
	Ctlr *ctlr;
    Trb *event_trb; 
    uint cycle_bit; 

	hp = arg;
	ctlr = hp->aux;
	
    ilock(ctlr);
   
    while(1){
        event_trb = (Trb *)ctlr->event_ring.curr; 
        cycle_bit = (CYCLE_BIT & event_trb->dwTrb3) ? 1 : 0;
        if(cycle_bit != ctlr->event_ring.cycle){
            ctlr->event_ring.cycle = cycle_bit;  
            break; 
        }

        _dump_trb(event_trb);
        
        int handled = 0;     
        uint trb_type = TYPE_GET(event_trb->dwTrb3);
        switch(trb_type){
            case EVENT_PORT_STS_CHANGE:
                __ddprint("received a port ststus change event\n");
                handleattach(hp, event_trb); 
                handled = 1; 
                break; 
            case EVENT_CMD_COMPLETE:
                __ddprint("received a command complete event\n");
                handlecmd(hp, event_trb); 
                handled = 1; 
                break;
            default: 
                __ddprint("received an unknown event\n");
                handled = 1; 
                break;
        }

        ctlr->event_ring.curr += sizeof(struct Trb); 
        if(handled) break; 
    }

    __ddprint("event handled\n");
    
    /* Clear the Event interrupt bit first to avoid the race condition
     * specified in the USB3 specs
     */
    xhcireg_wr(ctlr, ctlr->oper.usbsts, 0x8, 8); // EINT
    xhcireg_wr(ctlr, ctlr->runt.iman, 0x1, 1);

	iunlock(ctlr);
    return;
}

static void
scanpci(void)
{
    static int already = 0;
    uint bar;
    int i;
    Ctlr *ctlr;
    Pcidev *p;

    if(already)
        return;
  
    already = 1;
    p = nil;
    while(p = pcimatch(p, 0, 0)){

        /*
         * XHCI controllers programming interface = 0x30
         */
        if(p->ccrb != Pcibcserial || p->ccru != Pciscusb || p->ccrp != 0x30)
            continue;

        /* XHCI implements bar0 and bar1 (64b) 
         * FIXME Bug #6
         * What to do on 64 bit machines?
         */
        bar = p->mem[0].bar & ~0x0F;

        if(bar == 0){
            __ddprint("xhci: %#ux %#ux: failed to map registers\n", p->vid, p->did);
            continue;
        } else {
            __ddprint("xhci: vid:%#ux did:%#ux: successfully mapped registers \
                at %#ux size: %#ux\n", p->vid, p->did, bar, p->mem[0].size);
        }
  
        ctlr = malloc(sizeof(Ctlr));
        if(ctlr == nil)
            panic("xhci: out of memory");
  
        ctlr->xhci = vmap(bar, p->mem[0].size);
        __ddprint("vmap returned\n");
        if(ctlr->xhci == nil)
            panic("xhci: cannot map MMIO from PCI");
  
        if(p->intl == 0xFF || p->intl == 0){
            __ddprint("usbxhci: no irq assigned for bar %#ux\n", bar);
            continue;
        }

        __ddprint("xhci: %#ux %#ux: bar %#ux size %#x irq %d\n", p->vid, p->did, 
            bar, p->mem[0].size, p->intl);

        ctlr->pcidev = p;
 
        for(i = 0; i < Nhcis; i++){
            if(ctlrs[i] == nil){
                ctlrs[i] = ctlr;
                break;
            }
        }

        if(i == Nhcis)
            __ddprint("xhci: bug: no more controllers\n");
    }
}

/********************************************************
 ** Dead but possibly useful code
 *******************************************************/
// This code allocates slot contexts 
// Possibly used during device enumeration

//    // allocate memory for slot (context + MaxSlotsEn) * Packed32B
//    slotCtx *slot_ctx = xspanalloc(sizeof(struct SlotCtx), _64B, _64B);
//    packed32B *packed_slot = xspanalloc(sizeof(struct Packed32B), _64B, _64B);
//    
//    // for ep contexts
//    //uint num_ep = XHCI_MAXSLOTSEN * 2; // FIXME == 4 for now
//    uint num_ep = 4;
//    epCtx *ep_ctx[5];  // +1 for ep0 base dir
//    packed32B *packed_ep[5]; 
//    for(int i = 0; i < 5; i++) {
//        ep_ctx[i] = (epCtx *)xspanalloc(sizeof(struct EpCtx), _64B, _64B);
//        packed_ep[i] = (packed32B *)xspanalloc(sizeof(struct Packed32B), _64B, _64B);
//    }
//    
//    // configure and pack slot context
//    setup_default_slot_ctx(slot_ctx); 
//    pack_slot_ctx(packed_slot, slot_ctx);
//    
//
//    // configure and pack ep context
//    setup_default_ep_ctx(ep_ctx[0]); 
//    // mark ep in/out 
//    pack_ep_ctx(packed_ep[0], ep_ctx[0]);
//
//    // ep1 OUT
//    setup_default_ep_ctx(ep_ctx[1]); 
//    pack_ep_ctx(packed_ep[1], ep_ctx[1]);
//    
//    // ep1 IN
//    setup_default_ep_ctx(ep_ctx[1]); 
//    pack_ep_ctx(packed_ep[1], ep_ctx[1]);
//    
//    // ep2 OUT
//    setup_default_ep_ctx(ep_ctx[1]); 
//    pack_ep_ctx(packed_ep[1], ep_ctx[1]);
//    
//    // ep2 IN
//    setup_default_ep_ctx(ep_ctx[1]); 
//    pack_ep_ctx(packed_ep[1], ep_ctx[1]);

//    dcbaap[0] = packed_slot;
//    dcbaap[1] = packed_ep[0];
//    dcbaap[2] = packed_ep[1];
//    dcbaap[3] = packed_ep[2];
//    dcbaap[4] = packed_ep[3];
//    dcbaap[5] = packed_ep[4];


/** @brief Initializes hardware data structures used by the XHC and save the 
 *  references in software structure
 *  It will do: 
 *  1. allocates the Device Context Base Address Arrays
 *  2. setup event segment and event ring
 *  3. setup the command ring
 *
 *  @param[in/out] ctlr The software structure storing the hw pointers
 **/
static void
xhcimeminit(Ctlr *ctlr)
{
    packed32B **dcbaap = (packed32B **)mallocalign((sizeof(packed32B *) * (1+XHCI_MAXSLOTSEN) * 2), _64B, 0, 0); 
    memset((void *)dcbaap, 0, (sizeof(packed32B *) * (1+XHCI_MAXSLOTSEN) * 2));
    ctlr->devctx_bar = ((uint)PCIWADDR(dcbaap) & 0xFFFFFFFF); 
    
    /* setup one event ring segment tables (has one entry with 16 TRBs) for 
     * one interrupter 
     */
    Trb *event_ring_bar = (Trb *)mallocalign(sizeof(struct Trb) * 16, _4KB, 0, 0); 
    eventSegTabEntry *event_segtable = (eventSegTabEntry *)mallocalign(sizeof(struct EventSegTabEntry), _64B, 0, 0); 
    memset((void *)event_segtable, 0, sizeof(struct EventSegTabEntry));
    event_segtable->ringSegBarLo = (uint) PCIWADDR(event_ring_bar);
    event_segtable->ringSegSize = 16;

    ctlr->event_segtable.phys = (uint) PCIWADDR(event_segtable);
    ctlr->event_segtable.virt = (uint)event_segtable;
    ctlr->event_segtable.length = 1;
   
    /* FIXME Bug #7
     * Merge the deq pointer with the Sw_ring structure
     */
    ctlr->event_ring.phys = (uint) PCIWADDR(event_ring_bar); 
    ctlr->event_ring.virt = (uint)event_ring_bar;
    ctlr->event_ring.curr = (uint)event_ring_bar;
    ctlr->event_ring.length = 16;
    memset((void *)ctlr->event_ring.virt, 0, sizeof(struct Trb) * 16);
    __ddprint("physaddr for event ring deq  %#ux\n", (uint)ctlr->event_ring.phys);
    ctlr->event_ring.cycle = 1; 

    __ddprint("event ring allocation done\n");
    
    /* allocate the command ring */
    Trb *cmd_ring_bar = (Trb *)mallocalign((sizeof(struct Trb) * CMD_RING_SIZE), _4KB, 0, 0); 
    ctlr->cmd_ring.phys = (uint)PCIWADDR(cmd_ring_bar);
    ctlr->cmd_ring.virt = (uint)cmd_ring_bar;
    ctlr->cmd_ring.curr = (uint)cmd_ring_bar;
    ctlr->cmd_ring.length = CMD_RING_SIZE;
    ctlr->cmd_ring.cycle = 1;
}


static void
xhcireset(Ctlr *ctlr)
{
    int i; 
    ilock(ctlr);
    
    __ddprint("xhci with bar = %#ux reset\n", (uint)ctlr->xhci);
    xhcireg_wr(ctlr, ctlr->oper.usbcmd, 0x2, 2);
    
    i = 0; 
    /* Poll on run bit 0 for ready */
    uint usbsts = ctlr->oper.usbcmd; 
    while(xhcireg_rd(ctlr, usbsts, 0x800) != 0){
        delay(1);
        if((i = i + 1) == 100){
            /* FIXME Bug #8 
             * Clean way to report critical errors
             */
            panic("xhci controller reset timed out\n");
            break; 
        }
    }

    iunlock(ctlr);
    return;
}

static void
setdebug(Hci*, int d)
{
    debug = d;
}
    
static void
shutdown(Hci *hp)
{
    __ddprint("shutdown called\n");
    return;
}

/********************************
 * Entry point into XHCI driver *
 ********************************/

/** Looks for and reset the controller; 
 *  Registers all the top-level interface functions
 **/
static int
reset(Hci *hp) 
{
    static Lock resetlck;
    int i;
    Ctlr *ctlr;
    Pcidev *p;

    if(getconf("*nousbxhci"))
        return -1; 

    ilock(&resetlck);
    scanpci();

    /*
     * Any adapter matches if no hp->port is supplied,
     * otherwise the ports must match.
     */
    ctlr = nil;
    for(i = 0; i < Nhcis && ctlrs[i] != nil; i++){
        ctlr = ctlrs[i];
        if(ctlr->active == 0) 
        if(hp->port == 0 || hp->port == (uint)ctlr->xhci){
            ctlr->active = 1;
            break;
        }
    }
    
    iunlock(&resetlck);
    if(ctlrs[i] == nil || i == Nhcis)
        return -1;

    p = ctlr->pcidev;
    hp->aux = ctlr;
    hp->port = (uint)ctlr->xhci;
    hp->irq = p->intl;
    hp->tbdf = p->tbdf;
    hp->nports = 2; /* default */

    ctlr->caplength = xhcireg_rd(ctlr, CAPLENGTH_OFF, 0xFF);; 
    ctlr->num_port = xhcireg_rd(ctlr, HCSPARAMS1_OFF, 0xFF000000) >> 24;
   


    // setup all MMIO registers
    uint oper = ctlr->caplength;
    setup_oper(ctlr, oper);
    uint runt = xhcireg_rd(ctlr, RTS_OFF, 0xFFFFFFE0); 
    setup_runt(ctlr, runt);
    
    ctlr->db_off = xhcireg_rd(ctlr, DB_OFF, 0xFFFFFFFC);
    ctlr->max_slot = 2;

    __ddprint("usbxhci: caplength %d num_port %d\n", ctlr->caplength, ctlr->num_port);
    __ddprint("CAP base 0x%#ux OPER base 0x%#ux RUNT base 0x%#ux\n", (uint)ctlr->xhci, ctlr->oper.base, ctlr->runt.base);
    
    /* this call resets the chip and wait until regs are writable */
    __ddprint("going to send hardware reset\n"); 
    xhcireset(ctlr);

    /* this call initializes data structures */
    __ddprint("going to init memory structure\n"); 
    xhcimeminit(ctlr);

    /* now write all the registers */
    __ddprint("configuring internal registers\n"); 
    
    /* MAX_SLOT_EN == 2 TODO #3 */
    xhcireg_wr(ctlr, ctlr->oper.config, 0xFF, ctlr->max_slot);
    __ddprint("readback: MAX_SLOT_EN: %d should be 2\n", xhcireg_rd(ctlr, ctlr->oper.config, 0xFF));

    xhcireg_wr(ctlr, ctlr->oper.dcbaap_lo, 0xFFFFFFC0, ctlr->devctx_bar);
    xhcireg_wr(ctlr, ctlr->oper.dcbaap_hi, 0xFFFFFFFF, 0);
    __ddprint("configured device contexts\n"); 
   
    /* set up the event segment table size == 1 TODO #2 */
    xhcireg_wr(ctlr, ctlr->runt.erstsz, 0xFFFF, EVENT_SEGTABLE); 
    __ddprint("configured event segment table size %d\n", xhcireg_rd(ctlr, ctlr->runt.erstsz, 0xFFFF)); 
    
    xhcireg_wr(ctlr, ctlr->runt.erdp_lo, 0xFFFFFFF0, ctlr->event_ring.phys);
    xhcireg_wr(ctlr, ctlr->runt.erdp_hi, 0xFFFFFFFF, 0); 
    __ddprint("configured event ring deq ptr %#ux\n", (uint)xhcireg_rd(ctlr, ctlr->runt.erdp_lo, 0xFFFFFFFF)); 

    xhcireg_wr(ctlr, ctlr->runt.erstba_lo, 0xFFFFFFC0, ctlr->event_segtable.phys);
    xhcireg_wr(ctlr, ctlr->runt.erstba_hi, 0xFFFFFFFF, 0);
    __ddprint("configured event segtable bar%#ux\n", (uint)xhcireg_rd(ctlr, ctlr->runt.erstba_lo, 0xFFFFFFFF)); 
    
    /* set interrupt enable = 1
     */
    xhcireg_wr(ctlr, ctlr->runt.iman, 0x2, 2);
    __ddprint("interrupt is on\n"); 

    /* write 1 as initial value for cmd ring cycle bit */
    xhcireg_wr(ctlr, ctlr->oper.crcr_lo, 0x1, ctlr->cmd_ring.cycle);
    xhcireg_wr(ctlr, ctlr->oper.crcr_lo, 0xFFFFFFC0, ctlr->cmd_ring.phys);
    xhcireg_wr(ctlr, ctlr->oper.crcr_hi, 0xFFFFFFFF, 0);

    /* tell the controller to run */
    xhcireg_wr(ctlr, ctlr->oper.usbcmd, 0x4, 4);
    __ddprint("turn on host interrupt\n"); 
    xhcireg_wr(ctlr, ctlr->oper.usbcmd, 0x1, 1);
    __ddprint("controller is on\n"); 

    /*
     * Linkage to the generic HCI driver.
     */
    __ddprint("linking to generic HCI driver\n"); 
    hp->init = init;
    hp->dump = dump;
    hp->interrupt = interrupt;
    hp->epopen = epopen;
    hp->epclose = epclose;
    hp->epread = epread;
    hp->epwrite = epwrite;
    hp->seprintep = seprintep;
    hp->portenable = portenable;
    hp->portreset = portreset;
    hp->portstatus = portstatus;
    hp->shutdown = shutdown;
    hp->debug = setdebug;
    hp->type = "xhci";


    return 0;
}

void
usbxhcilink(void)
{
    addhcitype("xhci", reset);
}
