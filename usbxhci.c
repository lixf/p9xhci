/*
 * USB eXtensible Host Controller Interface driver.
 *
 */


/** 
 *  1. when is debug() called?
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


#define __ddprint if(debug) print
//#define diprint if(debug || iso->debug) print
//#define ddiprint if(debug>1 || iso->debug>1) print
//#define dqprint if(debug || (qh->io && qh->io->debug)) print
//#define ddqprint if(debug>1 || (qh->io && qh->io->debug>1)) print

/* static values -- read from CAPREG */
static uint caplength; 
static uint runtime_off; 
int debug = 1;

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
#define OPREG_OFF caplength 
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
#define DB_OFF (0x14)

/* runtime registers */
#define RTS_OFF (0x18)              // Runtime Space offset (from Bar)
#define RTREG_OFF runtime_off       // Offset of start of Runtime Registers
#define IMAN_OFF (RTREG_OFF + 0x20) // Interrupt management register
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
struct Sw_ring {
    // defines the fields for a TRB ring
    uint phys;  
    uint virt;
    uint curr;  // the current pointer into the structure
    char cycle; 
    uint length; 
};

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
    uint db_off; 
    uint max_slot;

    /* software values */
    // set up values
    uint devctx_bar; 
    struct Sw_ring cmd_ring;  
    struct Sw_ring event_ring;
    struct Sw_ring event_segtable;
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
    volatile uint ringSegBarLo;
    volatile uint ringSegBarHi; 
    volatile short rsvd1; 
    volatile unsigned short ringSegSize;  // possible sizes: 16-4096
    volatile uint rsvd2; 
}; 



struct Packed32B {
    volatile uint words[8]; 
};

struct Packed16B {
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


static void inline
ring_bell(Ctlr *ctlr, uint index, uint db) {
    assert(index < ctlr->max_slot);
    xhcireg_wr(ctlr, (ctlr->db_off + index * sizeof(uint)), 0xFFFFFFFF, db);
}


// TRB functions
static void inline 
send_command(Ctlr *ctlr, Trb *trb) {
    
    // what if there's some unfinished commands before the new command? TODO
    // ctlr should probably have a index pointer to the next free slot..
    if (ctlr == nil || ctlr->cmd_ring.virt == 0 || trb == nil) {
        panic("xhci send command internal error\n");
    } else {
        // FIXME how do I use memcpy??
        Trb *dst = (Trb *)ctlr->cmd_ring.virt;
        Trb *src = (Trb *)trb;
        dst->qwTrb0 = src->qwTrb0;
        dst->dwTrb2 = src->dwTrb2;
        dst->dwTrb3 = src->dwTrb3;
    }
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
_dump_trb(Trb *t) {
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
_dump_cmd_ring(struct Sw_ring *ring) {
    Trb *current; 
    __ddprint("***debug dump of command ring***\n");
    __ddprint("phys: %#ux, virt: %#ux, curr: %#ux, length: %#ux\n", 
        ring->phys, ring->virt, ring->curr, ring->length);
    for (uint i = 0; i < ring->length; i++) {
        current = (Trb *)(ring->virt + i * sizeof(struct Trb)); 
        _dump_trb(current);  
    }
}

static void
_dump_event_ring(struct Sw_ring *ring) {
    Trb *current; 
    __ddprint("***debug dump of event ring***\n");
    __ddprint("phys: %#ux, virt: %#ux, curr: %#ux, length: %#ux\n", 
        ring->phys, ring->virt, ring->curr, ring->length);
    for (uint i = 0; i < ring->length; i++) {
        current = (Trb *)(ring->virt + i * sizeof(struct Trb)); 
        _dump_trb(current);  
    }
}

static void
_dump_event_segtable(struct Sw_ring *ring) {
    __ddprint("***debug dump of event segment table***\n");
    __ddprint("phys: %#ux, virt: %#ux, curr: %#ux, length: %#ux\n", 
        ring->phys, ring->virt, ring->curr, ring->length);
}


static void
dump(Hci *hp) {
    Ctlr *ctlr;    
    ctlr = hp->aux;
    __ddprint("***debug dump of ctlr sw state***\n");
    __ddprint("active: %d, xhci: %#ux, oper: %#ux, runt: %#ux\n", 
        ctlr->active, (uint)ctlr->xhci, ctlr->oper, ctlr->runt);
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
/** @brief Some things need to be done here instead of the reset function
 *  TODO find out what those things are
 *  In OHCI: 
 *      1. enable interrupts
 *      2. enable all the ports
 *      3. set frames
 **/
static void
init(Hci *hp)
{
    __ddprint("xhci init\n");
    return; 
}


/** @brief This function sends a software reset to the port
 *  Currently this is used for enablement of USB2 device compatibility.
 *  Not sure when this is called from devusb layer. 
 *
 *  @param[in] port The port number to reset.
 *  @param[in] on This is used only to conform with the USBD interface. 
 **/
static int
portreset(Hci *hp, int port, int on)
{
    __ddprint("xhci portreset\n");
    Ctlr *ctlr;
    
    if(on == 0) {
        return 0;
    }
    
    ctlr = hp->aux;
    qlock(&ctlr->portlck);
    if(waserror()){
    	qunlock(&ctlr->portlck);
    	nexterror();
    }
    // now send the reset command to the port controlling register
    uint port_offset = PORTSC_OFF + port * PORTSC_ENUM_OFF; 
    xhcireg_wr(ctlr, port_offset, 0x10, 0x10); 
    int wait = 0;
    while (xhcireg_rd(ctlr, port_offset, 0x10)) {
        delay(10);
        wait++; 
        if (wait == 100) {
            __ddprint("xhci port %d reset timeout", port);
            qunlock(&ctlr->portlck);
            return -1;
        }
    }

    qunlock(&ctlr->portlck);
    return 0;
}



/** @berief Enabling a port is ignored, disabling a port is issued
 *  
 *  The USB3.0 spec does not allow/require software to enable a port, unlike
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
    if (on == 1) {
        __ddprint("xhci cannot allow enabling of a port\n");
        qunlock(&ctlr->portlck);
        return -1;
    } else {
        // send a disable command to PORTSTS register
        uint port_offset = PORTSC_OFF + port * PORTSC_ENUM_OFF; 
        uint port_sts = xhcireg_rd(ctlr, port_offset, 0x2);  //read the port enable bit
        if (port_sts == 0) {
            qunlock(&ctlr->portlck);
            return 0; 
        } else {
            xhcireg_wr(ctlr, port_offset, 0x2, 0x2);
            delay(5); 
            if (xhcireg_rd(ctlr, port_offset, 0x2)) {
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
	/*
	 * We must return status bits as a
	 * get port status hub request would do.
	 */

    /*
     * What's the format??
     */
	Ctlr *ctlr;
	ctlr = hp->aux;
    
    uint port_offset = PORTSC_OFF + port * PORTSC_ENUM_OFF; 
    uint port_sts = xhcireg_rd(ctlr, port_offset, 0xFFFFFFFF);
    __ddprint("xhci portstatus %#ux for port num %d\n", port_sts, port);
    //return port_sts; TODO
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


/** @brief this function handles the port status change event TRB
 *  This function detects if the port is in polling state for USB2 devices
 *  and initiate the port reset sequence to verify the correct state transition
 *  
 *  @param psce Port Status Change Event trb
 **/
#define PORTSTS_PLS 0x1E0 // [8:5]
#define PLS_POLLING 0x7
#define PLS_RXDETECT 0x5
static void 
handle_attachment(Hci *hp, Trb *psce) {
    Ctlr *ctlr;
    uint port_sts; 
    
    ctlr = hp->aux;
    // look for which port caused the attachment event
    uint port_id = psce->qwTrb0 >> 24 & 0xFF;
    __ddprint("port id %u caused attachment event\n", port_id);

    uint port_offset = PORTSC_OFF + port_id * PORTSC_ENUM_OFF; 
    
    // read port status
#ifdef XHCI_DEBUG
    port_sts = xhcireg_rd(ctlr, port_offset, 0xFFFFFFFF); 
    __ddprint("port status %#ux\n", port_sts);
#endif

    // read port link state to detect USB2 devices
    if (xhcireg_rd(ctlr, port_offset, PORTSTS_PLS) == PLS_POLLING) {
        // this is a USB2 device
        portreset(hp, port_id, 1);
    }

    if ((xhcireg_rd(ctlr, port_offset, PORTSTS_PLS) >> 5)  > 3) {
        __ddprint("USB device is not in the correct state\n");
    }

    // now issue an Enable Slot Command
    struct Trb slot_cmd; 
    slot_cmd.qwTrb0 = 0;
    slot_cmd.dwTrb2 = 0; 
    uint trb3 = TYPE_SET(9);                // enable slot
    // TODO trb3 |= EP_SET(ctlr->ext_cap.slot_type);// slot type
    trb3 |= ctlr->cmd_ring.cycle;           // cycle bit
    slot_cmd.dwTrb3 = trb3; 
    
    send_command(ctlr, &slot_cmd);    
    
    // now ring the door bell for the XHC
    uint db = 0; // db stream == 0, db target == 0
    ring_bell(ctlr, 0, db); 

    return; 
}

/** @brief This function handles the command completion event
 *  
 *
 **/
static void
handle_cmd_complete(Hci *hp, Trb *trb) {
    Ctlr *ctlr;
    ctlr = hp->aux;
    
    // get the assigned slot ID
    uint slot_id = trb->dwTrb3 >> 24 & 0xFF; 
    __ddprint("USB slot assignment command returned slot ID: %#ux\n", slot_id);
    
    // potentially need to return the slot ID to map it with the physical port TODO
    return; 
}




/** @brief This is the xHCI interrupt handler
 *  It will print out the interrupt status and check the event ring
 **/
static void
interrupt(Ureg*, void *arg)
{
    __ddprint("xhci interrupt\n");
	Hci *hp;
	Ctlr *ctlr;
	//ulong status; 
    Trb *event_trb; 
    uint cycle_bit; 

	hp = arg;
	ctlr = hp->aux;
	
    // lock here -- start checking event ring
    ilock(ctlr);
   
    // TODO remove: check for interrupt pending bit
    assert(xhcireg_rd(ctlr, IMAN_OFF, 0x1) == 1);
    while (1) {
#ifdef XHCI_DEBUG
        _dump_event_ring(&(ctlr->event_ring));
        _dump_event_segtable(&(ctlr->event_segtable));
#endif
        // process all the events until the cycle bit differs
        event_trb = (Trb *)ctlr->event_ring.curr; 
        // check cycle bit before processing
        cycle_bit = (CYCLE_BIT & event_trb->dwTrb3) ? 1 : 0;
        if (cycle_bit != ctlr->event_ring.cycle) {
            ctlr->event_ring.cycle = cycle_bit;  
            break; 
        }
        
        // now process this event

#ifdef XHCI_DEBUG
        _dump_trb(event_trb);
#endif
        int handled = 0;     
        uint trb_type = TYPE_GET(event_trb->dwTrb3);
        switch (trb_type) {
            case 34: // port status change
                handle_attachment(hp, event_trb); 
                handled = 1; 
                break; 
            case 33: // command complete
                // TODO handle
                __ddprint("received a command complete event\n");
                handle_cmd_complete(hp, event_trb); 
                handled = 1; 
                break;
            default: 
                __ddprint("received an unknown event\n");
                handled = 1; 
                break;
        }

        ctlr->event_ring.curr += sizeof(struct Trb); 
        if (handled) break; 
    }

    // clear the interrupt pending bit
    xhcireg_wr(ctlr, IMAN_OFF, 0x1, 1); // IP = 0 <-- this register is a RW1C

	iunlock(ctlr);
    return;
}

/*************************************************
 * Various helpers called by top-level functions * 
 *************************************************/

/** @brief Scans the PCIs to find the controller
 *  Called by reset. It will do this: 
 *  1. searches the pci dev struct chain to look for xhci controller
 *  2. vmaps the PCI bar for MMIO space
 *  3. checks the IRQ allocation
 *  4. expose the pci struct to the global context
 *  -- If any the above fails, it is not supported by plan 9 -- 
 **/
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
    // start enumerating everything on PCI
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
            __ddprint("xhci: %#ux %#ux: failed to map registers\n", p->vid, p->did);
            continue;
        } else {
            __ddprint("xhci: vid:%#ux did:%#ux: successfully mapped registers \
                at %#ux size: %#ux\n", p->vid, p->did, bar, p->mem[0].size);
        }
  
        ctlr = malloc(sizeof(Ctlr));
        if (ctlr == nil)
            panic("xhci: out of memory");
  
        ctlr->xhci = vmap(bar, p->mem[0].size);
        __ddprint("vmap returned\n");
        if (ctlr->xhci == nil) {
            panic("xhci: cannot map MMIO from PCI");
        }
  
        if(p->intl == 0xFF || p->intl == 0){
            __ddprint("usbxhci: no irq assigned for bar %#ux\n", bar);
            continue;
        }

        __ddprint("xhci: %#ux %#ux: bar %#ux size %#x irq %d\n", p->vid, p->did, 
            bar, p->mem[0].size, p->intl);

        ctlr->pcidev = p;
 
        // register this controller to ctlrs[], which is globle
        for(i = 0; i < Nhcis; i++) {
            if(ctlrs[i] == nil) {
                ctlrs[i] = ctlr;
                break;
            }
        }

        // Nhcis == 16 defined in usb.h
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
    // allocate the DCBAAP TODO probably use PCIWADDR
    packed32B **dcbaap = (packed32B **)mallocalign((sizeof(packed32B *) * (1+XHCI_MAXSLOTSEN) * 2), _64B, 0, 0); 
    memset((void *)dcbaap, 0, (sizeof(packed32B *) * (1+XHCI_MAXSLOTSEN) * 2));
    ctlr->devctx_bar = ((uint)PCIWADDR(dcbaap) & 0xFFFFFFFF); 
    
    // setup one event ring segment tables (has one entry with 16 TRBs) for one interrupter
    Trb *event_ring_bar = (Trb *)mallocalign(sizeof(struct Trb) * 16, _4KB, 0, 0); 
    eventSegTabEntry *event_segtable = (eventSegTabEntry *)mallocalign(sizeof(struct EventSegTabEntry), _64B, 0, 0); 
    memset((void *)event_segtable, 0, sizeof(struct EventSegTabEntry));
    event_segtable->ringSegBarLo = (uint) PCIWADDR(event_ring_bar);
    event_segtable->ringSegSize = 16;

    ctlr->event_segtable.phys = (uint) PCIWADDR(event_segtable);
    ctlr->event_segtable.virt = (uint)event_segtable;
    ctlr->event_segtable.length = 1;
    
    // Do I actually need this info? No, but it will make it cleaner
    // set deq ptr to the first trb in the event ring
    ctlr->event_ring.phys = (uint) PCIWADDR(event_ring_bar); 
    ctlr->event_ring.virt = (uint)event_ring_bar;
    ctlr->event_ring.curr = (uint)event_ring_bar;
    ctlr->event_ring.length = 16;
    memset((void *)ctlr->event_ring.virt, 0, sizeof(struct Trb) * 16);
    __ddprint("physaddr for event ring deq  %#ux\n", (uint)ctlr->event_ring.phys);
    ctlr->event_ring.cycle = 0; 

    __ddprint("event ring allocation done\n");
    
    // allocate the command ring
    Trb *cmd_ring_bar = (Trb *)mallocalign((sizeof(struct Trb) * CMD_RING_SIZE), _4KB, 0, 0); 
    ctlr->cmd_ring.phys = (uint)PCIWADDR(cmd_ring_bar);
    ctlr->cmd_ring.virt = (uint)cmd_ring_bar;
    ctlr->cmd_ring.curr = (uint)cmd_ring_bar;
    ctlr->cmd_ring.length = 1;

    // setup doorbell array


}


/** @brief Reset the XHC
 *  Waits until the XHC is ready to run
 **/
static void
xhcireset(Ctlr *ctlr)
{
    int i; 
    ilock(ctlr);
    
    __ddprint("xhci with bar = %#ux reset\n", (uint)ctlr->xhci);
    xhcireg_wr(ctlr, USBCMD_OFF, USBCMD_RESET, 2);/* global reset */
    
    i = 0; 
    while (xhcireg_rd(ctlr, USBSTS_OFF, USBSTS_CNR) != USBSTS_CNR_READY) {
        // WAIT until timeout
        delay(1);
        if ((i = i + 1) == 100) {
            __ddprint("xhci controller reset timed out\n");
            break; 
        }
    }

    iunlock(ctlr);
    return;
}

/** @brief Returns the port with the new attachment
 *  This function is used for polling CCS register change on all the ports
 *  Currently only used for debugging interrupts
 **/
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
    ctlr->db_off = xhcireg_rd(ctlr, DB_OFF, 0xFFFFFFFC);
    ctlr->max_slot = 2;

#ifdef XHCI_DEBUG
    __ddprint("printing all capabilities\n");
    int j = 0; 
    for (; j < 8; j++) {
        __ddprint("cap[%d] = 0x%#ux\n", j, xhcireg_rd(ctlr, (j<<2), (uint)-1));
    }
#endif

    __ddprint("usbxhci: caplength %d num_port %d\n", caplength, ctlr->num_port);
    __ddprint("CAP base 0x%#ux OPER base 0x%#ux RUNT base 0x%#ux\n", (uint)ctlr->xhci, ctlr->oper, ctlr->runt);
    
    // this call resets the chip and wait until regs are writable
    __ddprint("going to send hardware reset\n"); 
    xhcireset(ctlr);
    // this call initializes data structures
    __ddprint("going to init memory structure\n"); 
    xhcimeminit(ctlr);

    // now write all the registers
    __ddprint("configuring internal registers\n"); 
    
    // MAX_SLOT_EN == 2
    xhcireg_wr(ctlr, CONFIG_OFF, CONFIG_MAXSLOTEN, ctlr->max_slot);
    __ddprint("readback: MAX_SLOT_EN: %d should be 2\n", xhcireg_rd(ctlr, CONFIG_OFF, CONFIG_MAXSLOTEN));

    // DCBAAP_LO = ctlr->devctx_bar
    xhcireg_wr(ctlr, DCBAAP_OFF, DCBAAP_LO, ctlr->devctx_bar);
    
    // DCBAAP_HI = 0
    xhcireg_wr(ctlr, (DCBAAP_OFF + 4), 0xFFFFFFFF, ZERO);
    __ddprint("configured device contexts\n"); 
   
    // set up the event ring size
    xhcireg_wr(ctlr, ERSTSZ_OFF, 0xFFFF, 1); // write 1 to event segment table size register
    __ddprint("configured event segment table size %d\n", xhcireg_rd(ctlr, ERSTSZ_OFF, 0xFFFF)); 
    
    xhcireg_wr(ctlr, ERDP_OFF, 0xFFFFFFF0, ctlr->event_ring.phys);
    xhcireg_wr(ctlr, ERDP_OFF + 4, 0xFFFFFFFF, 0); 
    __ddprint("configured event ring deq ptr %#ux\n", (uint)xhcireg_rd(ctlr, ERDP_OFF, 0xFFFFFFFF)); 

    xhcireg_wr(ctlr, ERSTBA_OFF, 0xFFFFFFC0, ctlr->event_segtable.phys); // [5:0] is reserved 
    xhcireg_wr(ctlr, ERSTBA_OFF + 4, 0xFFFFFFFF, 0); // ERSTBA_HI = 0
    __ddprint("configured event segtable bar%#ux\n", (uint)xhcireg_rd(ctlr, ERSTBA_OFF, 0xFFFFFFFF)); 
    
#ifdef XHCI_DEBUG
    dump(hp); 
#endif
    // set interrupt enable = 1
    xhcireg_wr(ctlr, IMAN_OFF, 0x3, 2); // IE = 1, IP = 0 -> 2'b10 = 2
    __ddprint("interrupt is on\n"); 

    // CRCR_CMDRING_LO = ctlr->cmd_ring_bar
    xhcireg_wr(ctlr, CRCR_OFF, CRCR_CMDRING_LO, ctlr->cmd_ring.phys);

    // CRCR_CMDRING_HI = 0
    xhcireg_wr(ctlr, (CRCR_OFF + 4), 0xFFFFFFFF, ZERO);

    // tell the controller to run
    xhcireg_wr(ctlr, USBCMD_OFF, USBCMD_INTE, 4);
    __ddprint("turn on host interrupt\n"); 
    xhcireg_wr(ctlr, USBCMD_OFF, USBCMD_RS, 1);
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
