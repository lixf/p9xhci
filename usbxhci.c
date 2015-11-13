/*
 * USB eXtensible Host Controller Interface driver.
 *
 */

#include"u.h"
#include"../port/lib.h"
#include"mem.h"
#include"dat.h"
#include"fns.h"
#include"io.h"
#include"../port/error.h"
#include"usb.h"

//#define diprint if(debug || iso->debug) print
//#define ddiprint if(debug>1 || iso->debug>1) print
//#define dqprint if(debug || (qh->io && qh->io->debug)) print
//#define ddqprint if(debug>1 || (qh->io && qh->io->debug>1)) print

/* static values -- read from CAPREG */
static uint caplength; 
static uint runtime_off; 
int debug = 0;

#define XHCI_DEBUG
/* Hard coded values */
#define XHCI_MAXSLOTSEN 2 // defines the max. num of slots enabled by sw
#define CMD_RING_SIZE 32  // defines the number of TRBs in a CMD ring

#define _64B 64
#define _4KB (4 << 10)

/*************************************************
 * Controls for reading/writing of the registers *
 *************************************************/
/* register offsets (from base address of Capability Register) */
/* operational register */
// this definitely does not work if we have > 1 controller
#define OPREG_OFF (caplength+1) // TODO will this work?
#define CONFIG_OFF (OPREG_OFF + 0x38)
#define USBSTS_OFF (OPREG_OFF + 0x04)
#define DCBAAP_OFF (OPREG_OFF + 0x30)
#define CRCR_OFF (OPREG_OFF + 0x18)
#define USBCMD_OFF (OPREG_OFF + 0x0)
#define PORTSC_OFF (OPREG_OFF + 0x400) // this is a base addr for all ports
#define PORTSC_ENUM_OFF 0x10

/* capability register */
#define CAPLENGTH_OFF (0x0)
#define HCSPARAMS1_OFF (0x4)

/* runtime registers */
#define RTS_OFF (0x18)              // Runtime Space offset (from Bar)
#define RTREG_OFF runtime_off       // Offset of start of Runtime Registers
#define INTE_OFF (RTREG_OFF + 0x20) // Interrupt enable bit
#define ERSTSZ_OFF (RTREG_OFF + 0x28)   // Event segment size
#define ERDP_OFF (RTREG_OFF + 0x30)     // Event ring dequeue
#define ERSTBA_OFF (RTREG_OFF + 0x38)   // Event sgement bar 


/* mask of each used bits for different control signals */
/* operational register */
#define CONFIG_MAXSLOTEN 0xFF
#define USBSTS_CNR 0x800
#define DCBAAP_LO 0xFFFFFFC0
#define CRCR_CMDRING_LO 0xFFFFFFC0
#define USBCMD_RS 0x1
#define USBCMD_RESET 0x2
#define USBCMD_INTE 0x4
#define PORTSC_CCS 0x1 // current connect status

/* capability register */
#define CAPLENGTH 0xFF
#define HCSPARAMS1_MAXPORT 0xFF000000

/* actual values of those control signals */
#define ZERO 0 // for 64bit -> 32bit hacking FIXME
/* operational register */
#define USBSTS_CNR_READY 0 

/* helper macros */
#define REPORT(s) print("*** xHCI: %s ***\n", s); 


/* declarations */
struct Ctlr;  
struct Trb;     // for constructing transfer ring
struct Td;      // for constructing transfer 
struct SlotCtx; // slot context
struct EpCtx;   // endpoint context 

struct Packed32B;   // Generic packed 32-byte -- used for context
struct Packed16B;   // Generic packed 16-byte -- used for TRBs
                    // can also use for event ring segment table
struct EventSegTabEntry;    // event ring segment table entry 

/* definitions */
struct Ctlr {
    Lock; 
    QLock portlck; 
    Pcidev *pcidev; 
    int active; 
    void *xhci; // MMIO pointer
    uint oper;
    uint runt;
    // throw away the rest for now
    // because XHCI has no QH or ISO support for now
    // read from hw
    uint caplength; 
    uint num_port; 

    /* software values */
    // set up values
    uint devctx_bar; 
    uint cmd_ring_bar;  
    uint event_deq; 
    uint event_segtable; 
    // runtime values
    uint event_cycle_bit;
};

struct Trb {
	volatile uvlong qwTrb0; // data buf lo + hi
#define DIR_IN_MASK      (0x80ULL << 0)
#define WLENGTH_MASK     (0xFFFFULL << 48)
	volatile uint dwTrb2;
#define ERROR_GET(x)     (((x) >> 24) & 0xFF)
#define ERROR_SET(x)     (((x) & 0xFF) << 24)
#define TDSZ_GET(x)      (((x) >> 17) & 0x1F)
#define TDSZ_SET(x)      (((x) & 0x1F) << 17)
#define REM_GET(x)       ((x) & 0xFFFFFF)
#define REM_SET(x)       ((x) & 0xFFFFFF)
#define BYTES_GET(x)     ((x) & 0x1FFFF)
#define BYTES_SET(x)     ((x) & 0x1FFFF)
#define IRQ_GET(x)       (((x) >> 22) & 0x3FF)
#define IRQ_SET(x)       (((x) & 0x3FF) << 22)
#define STREAM_GET(x)    (((x) >> 16) & 0xFFFF)
#define STREAM_SET(x)    (((x) & 0xFFFF) << 16)

    volatile uint dwTrb3;
#define TYPE_GET(x)      (((x) >> 10) & 0x3F)
#define TYPE_SET(x)      (((x) & 0x3F) << 10)
#define CYCLE_BIT        (1U << 0)
#define TC_BIT       (1U << 1)   /* command ring only */
#define ENT_BIT      (1U << 1)   /* transfer ring only */
#define ISP_BIT      (1U << 2)
#define NSNOOP_BIT       (1U << 3)
#define CHAIN_BIT        (1U << 4)
#define IOC_BIT      (1U << 5)
#define IDT_BIT      (1U << 6)
#define TBC_GET(x)       (((x) >> 7) & 3)
#define TBC_SET(x)       (((x) & 3) << 7)
#define BEI_BIT      (1U << 9)
#define DCEP_BIT     (1U << 9)
#define PRSV_BIT     (1U << 9)
#define BSR_BIT      (1U << 9)
#define TRT_MASK     (3U << 16)
#define TRT_NONE     (0U << 16)
#define TRT_OUT      (2U << 16)
#define TRT_IN       (3U << 16)
#define DIR_IN       (1U << 16)
#define TLBPC_GET(x)     (((x) >> 16) & 0xF)
#define TLBPC_SET(x)     (((x) & 0xF) << 16)
#define EP_GET(x)        (((x) >> 16) & 0x1F)
#define EP_SET(x)        (((x) & 0x1F) << 16)
#define FRID_GET(x)      (((x) >> 20) & 0x7FF)
#define FRID_SET(x)      (((x) & 0x7FF) << 20)
#define ISO_SIA_BIT      (1U << 31)
#define SUSP_EP_BIT      (1U << 23)
#define SLOT_GET(x)      (((x) >> 24) & 0xFF)
#define SLOT_SET(x)      (((x) & 0xFF) << 24)

/* Commands */
#define TYPE_RESERVED      0x00
#define TYPE_NORMAL        0x01
#define TYPE_SETUP_STAGE   0x02
#define TYPE_DATA_STAGE    0x03
#define TYPE_STATUS_STAGE  0x04
#define TYPE_ISOCH     0x05
#define TYPE_LINK      0x06
#define TYPE_EVENT_DATA    0x07
#define TYPE_NOOP      0x08
#define TYPE_ENABLE_SLOT   0x09
#define TYPE_DISABLE_SLOT  0x0A
#define TYPE_ADDRESS_DEVICE    0x0B
#define TYPE_CONFIGURE_EP  0x0C
#define TYPE_EVALUATE_CTX  0x0D
#define TYPE_RESET_EP      0x0E
#define TYPE_STOP_EP       0x0F
#define TYPE_SET_TR_DEQUEUE    0x10
#define TYPE_RESET_DEVICE  0x11
#define TYPE_FORCE_EVENT   0x12
#define TYPE_NEGOTIATE_BW  0x13
#define TYPE_SET_LATENCY_TOL   0x14
#define TYPE_GET_PORT_BW   0x15
#define TYPE_FORCE_HEADER  0x16
#define TYPE_NOOP_CMD      0x17

/* Events */
#define EVENT_TRANSFER     0x20
#define EVENT_CMD_COMPLETE 0x21
#define EVENT_PORT_STS_CHANGE  0x22
#define EVENT_BW_REQUEST       0x23
#define EVENT_DOORBELL     0x24
#define EVENT_HOST_CTRL    0x25
#define EVENT_DEVICE_NOTIFY    0x26
#define EVENT_MFINDEX_WRAP 0x27

/* Error codes */
#define ERROR_INVALID      0x00
#define ERROR_SUCCESS      0x01
#define ERROR_DATA_BUF     0x02
#define ERROR_BABBLE       0x03
#define ERROR_XACT     0x04
#define ERROR_TRB      0x05
#define ERROR_STALL        0x06
#define ERROR_RESOURCE     0x07
#define ERROR_BANDWIDTH    0x08
#define ERROR_NO_SLOTS     0x09
#define ERROR_STREAM_TYPE  0x0A
#define ERROR_SLOT_NOT_ON  0x0B
#define ERROR_ENDP_NOT_ON  0x0C
#define ERROR_SHORT_PKT    0x0D
#define ERROR_RING_UNDERRUN    0x0E
#define ERROR_RING_OVERRUN 0x0F
#define ERROR_VF_RING_FULL 0x10
#define ERROR_PARAMETER    0x11
#define ERROR_BW_OVERRUN   0x12
#define ERROR_CONTEXT_STATE    0x13
#define ERROR_NO_PING_RESP 0x14
#define ERROR_EV_RING_FULL 0x15
#define ERROR_INCOMPAT_DEV 0x16
#define ERROR_MISSED_SERVICE   0x17
#define ERROR_CMD_RING_STOP    0x18
#define ERROR_CMD_ABORTED  0x19
#define ERROR_STOPPED      0x1A
#define ERROR_LENGTH       0x1B
#define ERROR_BAD_MELAT    0x1D
#define ERROR_ISOC_OVERRUN 0x1F
#define ERROR_EVENT_LOST   0x20
#define ERROR_UNDEFINED    0x21
#define ERROR_INVALID_SID  0x22
#define ERROR_SEC_BW       0x23
#define ERROR_SPLIT_XACT   0x24
};

struct Td {
    volatile int id;             // FIXME TD identifier
    volatile struct Trb *trbs;   // The linked trb
    volatile int length;         // num of trb in this TD
}; 

struct SlotCtx {
    /* word 0 */
    uint routeStr;  // bits[19:00] route string
    char speed;     // bits[23:20] speed
    char mmt;       // bits[25] multi-TT TODO
    char hub;       // bits[26] 1 if device is hub
    char ctxEntry;  // bits[31:27] index of last valid entry
    
    /* word 1 */
    uint maxExit;   // bits[15:00] max exit latency in u-sec
    char rootPort;  // bits[23:16] Root hub port number
    char numPort;   // bits[31:24] if dev is hub, shows num of ports in dev

    /* word 2 */
    char tthubSlot;  // bits[7:0]   TT hub slot ID TODO
    char ttportNum;  // bits[15:8]  TT port number TODO
    char ttt;        // bits[17:16] TT think time TODO
    uint intrTarget; // bits[31:22] Interrupt target TODO 

    /* word 3 */
    char devAddr;   // bits[7:0]    USB device address
    char slotState; // bits[31:27]  Slot state -- written by hw

    /* word 4-7 reserved = zeros */
}; 

struct EpCtx {
    /* word 0 */
    char epstate;   // bits[2:0] current operational state
    char mult;      // bits[9:8] max num of burst TODO
    char maxPStream;// bits[14:10] Max Primary Streams TODO
    char lsa;       // bits[15] linear stream array TODO
    char interval;  // bits[23:16] interval
    char maxEsitHi; // bits[31:24] max ep service time interval payload hi TODO
    
    /* word 1 */
    char cErr;       // bits[2:1] Error count
    char epType;     // bits[5:3] Endpoint type
    char hostInitDis;// bits[7] Host initiate disable
    char maxBurst;   // bits[15:8] Max burst size
    uint maxPktSize; // bits[31:16] Max packet size

    /* word 2 */
    char deqCyState;// bits[0] Dequeue Cycle State
    uint trDeqPtrLo;// bits[31:4] TR dequeue pointer Low

    /* word 3 */
    uint trDeqPtrHi;// bits[31:0] TR dequeue pointer High

    /* word 4 */
    uint aveTrbLen; // bits[15:0] average TRB length
    char maxEsitLo; // bits[31:24] max ep service time interval payload lo TODO
}; 

struct EventSegTabEntry {
    volatile uvlong ringSegBar; // 64 bit event ring segment base address
    volatile uint ringSegSize;  // possible sizes: 16-4096
}; 



struct Packed32B {
    volatile uint words[8]; 
};

struct Packed16B {
    volatile uint words[4]; 
};

/* typedefs */
typedef struct Ctlr Ctlr; 
typedef struct Trb Trb; 
typedef struct Td Td; 
typedef struct SlotCtx slotCtx; 
typedef struct EpCtx epCtx; 
typedef struct Packed32B packed32B; 
typedef struct Packed16B packed16B; 
typedef struct EventSegTabEntry eventSegTabEntry; 

/*********************************************
 * xHCI specific functions
 *********************************************/
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
pack_slot_ctx(packed32B *packed, slotCtx *slots) {
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
unpack_slot_ctx(packed32B *packed, slotCtx *slots) {
    return; 
}

static inline void
pack_ep_ctx(packed32B *packed, epCtx *ep) {
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
unpack_ep_ctx(packed32B *packed, epCtx *ep) {
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
setup_default_slot_ctx(slotCtx *slot_ctx) {
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
setup_default_ep_ctx(epCtx *ep_ctx) {
    // TODO write something 
    ep_ctx->epstate       = 0;
    ep_ctx->hostInitDis   = 0;
    ep_ctx->trDeqPtrLo    = 0;
    ep_ctx->trDeqPtrHi    = 0;
    return; 
}

/** @brief This function will write to one of the registers
 *  @param TODO
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


// TRB functions
static void 
send_command() {
    return; 
}








/**********************************************
 * Top level interface functions -- debugging * 
 **********************************************/
static char*
seprintep(char *s, char *e, Ep *ep)
{
}

static void
dump(Hci *hp)
{
}


/**************************************************
 * Top level interface functions -- functionality * 
 **************************************************/
static void
init(Hci *hp)
{
    print("xhci init\n");
    return; 
}

static int
portreset(Hci *hp, int port, int on)
{
    print("xhci portreset\n");
    return -1; 
}

static int
portenable(Hci *hp, int port, int on)
{
    print("xhci portenable\n");
    return -1; 
}

static int
portstatus(Hci *hp, int port)
{
    print("xhci portstatus\n");
    return -1;
}

static void
epclose(Ep *ep)
{
    print("xhci epclose\n");
    return; 
}

static void
epopen(Ep *ep)
{
    print("xhci epopen\n");
    return; 
}

static long
epwrite(Ep *ep, void *a, long count)
{
    print("xhci epwrite\n");
    return -1;
}

static long
epread(Ep *ep, void *a, long count)
{
    print("xhci epread\n");
    return -1;
}


/** @brief this function handles the port status change event TRB
 *  
 *  @param psce Port Status Change Event trb
 **/
static void 
handle_attachment(Ctlr *ctlr, Trb *psce) {
    uint port_sts; 
  
    // look for which port caused the attachment event
    uint port_id = psce->qwTrb0 >> 24 & 0xFF;
    print("port id %u caused attachment event\n", port_id);

    // read port status
    uint port_offset = PORTSC_OFF + port_id * PORTSC_ENUM_OFF; 
    port_sts = xhcireg_rd(ctlr, port_offset, 0xFFFFFFFF); 
    print("port status 0x%#ux\n", port_sts);
    return; 
}


static void
dump_trb(Trb *t) {
    assert(t != nil); 
    print("qwTrb0 (data ptr low): 0x%#ux\n", (uint)(t->qwTrb0 & 0xFFFFFFFF));
    print("qwTrb0 (data ptr high): 0x%#ux\n", (uint)(t->qwTrb0 >> 32));
    print("dwTrb2 (status): 0x%#ux\n", t->dwTrb2);
    print("dwTrb3 (status): 0x%#ux\n", t->dwTrb3);
    print("cycle bit: %d\n", (t->dwTrb3 & CYCLE_BIT));
    return; 
}


/** @brief This is the xHCI interrupt handler
 *  It will print out the interrupt status and check the command ring
 **/
static void
interrupt(Ureg*, void *arg)
{
    print("xhci interrupt\n");
	Hci *hp;
	Ctlr *ctlr;
	//ulong status; 
    Trb *event_trb; 
    uint cycle_bit; 

	hp = arg;
	ctlr = hp->aux;
	
    // lock here -- start checking event ring
    ilock(ctlr);
    
    while (1) {
        // process all the events until the cycle bit differs
        event_trb = (Trb *)ctlr->event_deq; 
        // check cycle bit before processing
        cycle_bit = (CYCLE_BIT & event_trb->dwTrb3) ? 1 : 0;
        if (cycle_bit != ctlr->event_cycle_bit) {
            ctlr->event_cycle_bit = cycle_bit;  
            break; 
        }
        // now process this event
        print("received event TRB: \n");
        dump_trb(event_trb);

        // FIXME the only event we handle now is port connection status change
        //if (trb_type == EVENT_PORT_STS_CHANGE) {
        handle_attachment(ctlr, event_trb); 
        break;

        ctlr->event_deq += sizeof(struct Trb); 
    }

	iunlock(ctlr);
    return;
}

/*************************************************
 * Various helpers called by top-level functions * 
 *************************************************/

/** @brief Scans the PCIs to find the controller
 *  Called by reset
 **/
static void
scanpci(void)
{
    static int already = 0;
    uint bar;
    int i;
    Ctlr *ctlr;
    Pcidev *p; // defined in io.h as struct

    if(already)
        return;
  
    already = 1;
    p = nil;
    // start enumerating everything on PCI
    // pcimatch(vid, did) -- 0 for everything
    while (p = pcimatch(p, 0, 0)) {

        /*
         * XHCI controllers programming interface = 0x30
         */
        // ccrb -- (base class code) controller types
        // ccru -- (sub-class code)
        // ccrp -- programming interface class mode?? Not sure..
        if(p->ccrb != Pcibcserial || p->ccru != Pciscusb || p->ccrp != 0x30)
            continue;

        // XHCI implements bar0 and bar1 (64b)
        bar = p->mem[0].bar & ~0x0F;

        if(bar == 0){
            print("xhci: %#ux %#ux: failed to map registers\n", p->vid, p->did);
            continue;
        } else {
            print("xhci: vid:%#ux did:%#ux: successfully mapped registers at %#ux size: %#ux\n", 
                p->vid, p->did, bar, p->mem[0].size);
        }
  
        ctlr = malloc(sizeof(Ctlr));
        if (ctlr == nil)
            panic("xhci: out of memory");
  
        ctlr->xhci = vmap(bar, p->mem[0].size);
        print("vmap returned\n");
        if (ctlr->xhci == nil) {
            panic("xhci: cannot map MMIO from PCI");
        }
  
        if(p->intl == 0xFF || p->intl == 0){
            print("usbxhci: no irq assigned for bar %#ux\n", bar);
            continue;
        }

        print("xhci: %#ux %#ux: bar %#ux size %#x irq %d\n", p->vid, p->did, bar, 
            p->mem[0].size, p->intl);

        ctlr->pcidev = p;
 
        // register this controller to ctlrs[]
        for(i = 0; i < Nhcis; i++) {
            if(ctlrs[i] == nil) {
                ctlrs[i] = ctlr;
                break;
            }
        }

        // Nhcis == 16 defined in usb.h
        if(i == Nhcis)
            print("xhci: bug: no more controllers\n");
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

static void
xhcimeminit(Ctlr *ctlr)
{
    // allocate the DCBAAP
    packed32B **dcbaap = (packed32B **)xspanalloc((sizeof(packed32B *) * (1+XHCI_MAXSLOTSEN) * 2), _64B, _64B); 
    memset((void *)dcbaap, 0, (sizeof(packed32B *) * (1+XHCI_MAXSLOTSEN) * 2));
    ctlr->devctx_bar = ((uint)dcbaap & 0xFFFFFFFF); 
    
    // setup one event ring segment tables (has one entry with 16 TRBs) for one interrupter
    Trb *event_ring_bar = (Trb *)xspanalloc(sizeof(struct Trb) * 16, _4KB, _4KB); 
    eventSegTabEntry *event_segtable = (eventSegTabEntry *)xspanalloc(sizeof(struct EventSegTabEntry), _64B, _64B); 
    event_segtable->ringSegBar  = (uvlong)event_ring_bar;
    event_segtable->ringSegSize = 16;
    ctlr->event_segtable = (uint) event_segtable; 
    // set deq ptr to the first trb in the event ring
    ctlr->event_deq = (uint) event_ring_bar; 
    ctlr->event_cycle_bit = 0; 
    
    // allocate the command ring and set up the pointers
    Trb *cmd_ring_bar = (Trb *)xspanalloc((sizeof(struct Trb) * CMD_RING_SIZE), _4KB, _4KB); 
    ctlr->cmd_ring_bar = (uint)cmd_ring_bar; 
}
    
static void
xhcireset(Ctlr *ctlr)
{
    int i; 
    ilock(ctlr);
    
    print("xhci with bar = %#ux reset\n", (uint)ctlr->xhci);
    xhcireg_wr(ctlr, USBCMD_OFF, USBCMD_RESET, 2);/* global reset */
    
    i = 0; 
    while (xhcireg_rd(ctlr, USBSTS_OFF, USBSTS_CNR) != USBSTS_CNR_READY) {
        // WAIT until timeout
        delay(1);
        if ((i = i + 1) == 100) {
            print("xhci controller reset timed out\n");
            break; 
        }
    }

    iunlock(ctlr);
    return;
}

static int 
port_new_attach(Ctlr *ctlr)
{
    uint i; 
    // check all the ports
    for (i = 0; i < ctlr->num_port; i++) {
        if (xhcireg_rd(ctlr, (PORTSC_OFF+i*PORTSC_ENUM_OFF), PORTSC_CCS) == 1) {
            return i; 
        }
    }
    return -1; 
}
    
static void
setdebug(Hci*, int d)
{
    debug = d;
}
    
static void
shutdown(Hci *hp)
{
}

/********************************
 * Entry point into XHCI driver *
 ********************************/

/** @brief Looks for and reset the controller; registers
 *  all the top-level interface functions
 *  
 *  @param[in] hp The hci struct from devusb.c
 *  @return 0; or -1 if failed
 **/
static int
reset(Hci *hp) 
{
    static Lock resetlck;
    int i;
    Ctlr *ctlr;
    Pcidev *p;

    // some global configuration to turn off certain controller
    // see plan9ini[] in main.c
    if (getconf("*nousbxhci"))
        return -1; 

    ilock(&resetlck);
    scanpci();

    /*
     * Any adapter matches if no hp->port is supplied,
     * otherwise the ports must match.
     */
    // ctrls is global //
    // this is the port set by the scanpci function
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

    // copy things over to hp structure
    p = ctlr->pcidev;
    hp->aux = ctlr;
    hp->port = (uint)ctlr->xhci;
    hp->irq = p->intl;
    hp->tbdf = p->tbdf;
    // TODO change this?
    hp->nports = 2;/* default */

    // read some stuff and set global values
    // FIXME not work with > 1 controller
    caplength = xhcireg_rd(ctlr, CAPLENGTH_OFF, CAPLENGTH);
    runtime_off = xhcireg_rd(ctlr, RTS_OFF, 0xFFFFFFE0);
    ctlr->caplength = caplength; 
    ctlr->num_port = xhcireg_rd(ctlr, HCSPARAMS1_OFF, HCSPARAMS1_MAXPORT) >> 24;
    ctlr->oper = (uint)ctlr->xhci + caplength; 
    ctlr->runt = (uint)ctlr->xhci + runtime_off;

#ifdef XHCI_DEBUG
    print("printing all capabilities\n");
    int j = 0; 
    for (; j < 8; j++) {
        print("cap[%d] = 0x%#ux\n", j, xhcireg_rd(ctlr, (j<<2), (uint)-1));
    }
#endif

    print("usbxhci: caplength %d num_port %d\n", caplength, ctlr->num_port);
    print("CAP base 0x%#ux OPER base 0x%#ux RUNT base 0x%#ux\n", (uint)ctlr->xhci, ctlr->oper, ctlr->runt);
    
    // this call resets the chip and wait until regs are writable
    print("going to send hardware reset\n"); 
    xhcireset(ctlr);
    // this call initializes data structures
    print("going to init memory structure\n"); 
    xhcimeminit(ctlr);

    // now write all the registers
    print("configuring internal registers\n"); 
    
    // MAX_SLOT_EN == 2
    xhcireg_wr(ctlr, CONFIG_OFF, CONFIG_MAXSLOTEN, 2);
    print("readback: MAX_SLOT_EN: %d should be 2\n", xhcireg_rd(ctlr, CONFIG_OFF, CONFIG_MAXSLOTEN));

    // DCBAAP_LO = ctlr->devctx_bar
    xhcireg_wr(ctlr, DCBAAP_OFF, DCBAAP_LO, ctlr->devctx_bar);
    
    // DCBAAP_HI = 0
    xhcireg_wr(ctlr, (DCBAAP_OFF + 4), 0xFFFFFFFF, ZERO);
    print("configured device contexts\n"); 
   
    // set up the event ring size
    xhcireg_wr(ctlr, ERSTSZ_OFF, 0xFFFF, 1); // write 1 to event segment table size register
    print("configured event ring size\n"); 
    xhcireg_wr(ctlr, ERDP_OFF, 0xFFFFFFF0, ctlr->event_deq); // [2:0] used by xHC, [3] is Event Handle Busy TODO
    xhcireg_wr(ctlr, ERDP_OFF + 4, 0xFFFFFFFF, 0); 
    print("configured event ring deq ptr\n"); 
    xhcireg_wr(ctlr, ERSTBA_OFF, 0xFFFFFFC0, ctlr->event_segtable); // [5:0] is reserved 
    xhcireg_wr(ctlr, ERSTBA_OFF + 4, 0xFFFFFFFF, 0); // ERSTBA_HI = 0
    print("configured event ring bar\n"); 
    // enable interrupt and TODO: disable MSI/MSIX
    // set interrupt enable = 1
    xhcireg_wr(ctlr, INTE_OFF, 0x3, 2); // IE = 1, IP = 0 -> 2'b10 = 2
    print("interrupt is on\n"); 

    // CRCR_CMDRING_LO = ctlr->cmd_ring_bar
    xhcireg_wr(ctlr, CRCR_OFF, CRCR_CMDRING_LO, ctlr->cmd_ring_bar);

    // CRCR_CMDRING_HI = 0
    xhcireg_wr(ctlr, (CRCR_OFF + 4), 0xFFFFFFFF, ZERO);

    // tell the controller to run
    xhcireg_wr(ctlr, USBCMD_OFF, USBCMD_INTE, 1);
    print("turn on host interrupt\n"); 
    xhcireg_wr(ctlr, USBCMD_OFF, USBCMD_RS, 1);
    print("controller is on\n"); 
    /*
     * Linkage to the generic HCI driver.
     */
    print("linking to generic HCI driver\n"); 
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

    // poll for CCS = 1
    while(1) {
        delay(10);
        int new;
        if ((new = port_new_attach(ctlr)) != -1) {
            print("new device attached at %d\n", new);
            
            // dump the event TRB    
            event_trb = (Trb *)ctlr->event_deq; 
            // check cycle bit before processing
            cycle_bit = (CYCLE_BIT & event_trb->dwTrb3) ? 1 : 0;
            if (cycle_bit != ctlr->event_cycle_bit) {
                ctlr->event_cycle_bit = cycle_bit;  
                break; 
            }
            // now process this event
            print("received event TRB: \n");
            dump_trb(event_trb);            
        }
    }
    // TODO remove the test code


    return 0;
}

void
usbxhcilink(void)
{
    addhcitype("xhci", reset);
}
