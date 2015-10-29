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

#define INL(x) inl(ctlr->port+(x))
#define OUTL(x, v) outl(ctlr->port+(x), (v))
#define TRUNC(x, sz) ((x) & ((sz)-1))
//#define PTR(q) ((void*) KADDR((ulong)(q) & ~ (0xF|PCIWINDOW)))
#define PORT(p) (Portsc0 + 2*(p))
//#define diprint if(debug || iso->debug) print
//#define ddiprint if(debug>1 || iso->debug>1) print
//#define dqprint if(debug || (qh->io && qh->io->debug)) print
//#define ddqprint if(debug>1 || (qh->io && qh->io->debug>1)) print

/* static values -- read from CAPREG */
static uint caplength; 
int debug = 0;

/* Hard coded values */
#define XHCI_PCI_BAR 0
#define XHCI_MAXSLOTSEN 2
#define _64B 64

#define XHCI_CAPA 0
#define XHCI_OPER 1
#define XHCI_RUNT 2
#define XHCI_DOOR 3

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
    #define DCBAAP_HI_OFF 0x04
#define CRCR_OFF (OPREG_OFF + 0x18)
    #define CRCR_CMDRING_HI_OFF 0x04
#define USBCMD_OFF (OPREG_OFF + 0x0)
#define PORTSC_OFF (OPREG_OFF + 0x400) // this is a base addr for all ports
#define PORTSC_ENUM_OFF 0x10

/* capability register */
#define CAPLENGTH_OFF (0x0)
#define HCSPARAMS1_OFF (0x4)




/* mask of each used bits for different control signals */
/* operational register */
#define CONFIG_MAXSLOTEN 0xFF
#define USBSTS_CNR 0x800
#define DCBAAP_LO 0xFFFFFFC0
#define DCBAAP_HI 0xFFFFFFFF
#define CRCR_CMDRING_LO 0xFFFFFFC0
#define CRCR_CMDRING_HI 0xFFFFFFFF
#define USBCMD_RS 0x1
#define USBCMD_RESET 0x2
#define PORTSC_CCS 0x1 // current connect status

/* capability register */
#define CAPLENGTH 0xFF
#define HCSPARAMS1_MAXPORT 0xFF000000




/* actual values of those control signals */
#define ZERO 0 // for 64bit -> 32bit hacking FIXME
/* operational register */
#define USBSTS_CNR_READY 0 
#define USBCMD_RS_RUN 1 
#define USBCMD_RESET_RESET 2 
#define PORTSC_CCS_CONNECT 1

/* capability register */


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


/* definitions */
struct Ctlr {
    Lock; 
    QLock portlck; 
    Pcidev *pcidev; 
    int active; 
    int port; 
    uint oper;
    // throw away the rest for now
    // because XHCI has no QH or ISO support for now
    // read from hw
    uint caplength; 
    uint num_port; 

    // software values
    uint devctx_bar; 
    uint cmd_ring_bar;  
};

struct Trb {
    	//volatile uint64_t   qwTrb0;
	uint qwTrb0;
#define XHCI_TRB_0_DIR_IN_MASK      (0x80ULL << 0)
#define XHCI_TRB_0_WLENGTH_MASK     (0xFFFFULL << 48)
    	//volatile uint32_t   dwTrb2;
	uint dwTrb2;
#define XHCI_TRB_2_ERROR_GET(x)     (((x) >> 24) & 0xFF)
#define XHCI_TRB_2_ERROR_SET(x)     (((x) & 0xFF) << 24)
#define XHCI_TRB_2_TDSZ_GET(x)      (((x) >> 17) & 0x1F)
#define XHCI_TRB_2_TDSZ_SET(x)      (((x) & 0x1F) << 17)
#define XHCI_TRB_2_REM_GET(x)       ((x) & 0xFFFFFF)
#define XHCI_TRB_2_REM_SET(x)       ((x) & 0xFFFFFF)
#define XHCI_TRB_2_BYTES_GET(x)     ((x) & 0x1FFFF)
#define XHCI_TRB_2_BYTES_SET(x)     ((x) & 0x1FFFF)
#define XHCI_TRB_2_IRQ_GET(x)       (((x) >> 22) & 0x3FF)
#define XHCI_TRB_2_IRQ_SET(x)       (((x) & 0x3FF) << 22)
#define XHCI_TRB_2_STREAM_GET(x)    (((x) >> 16) & 0xFFFF)
#define XHCI_TRB_2_STREAM_SET(x)    (((x) & 0xFFFF) << 16)

    //volatile uint32_t   dwTrb3;
    uint dwTrb3;
#define XHCI_TRB_3_TYPE_GET(x)      (((x) >> 10) & 0x3F)
#define XHCI_TRB_3_TYPE_SET(x)      (((x) & 0x3F) << 10)
#define XHCI_TRB_3_CYCLE_BIT        (1U << 0)
#define XHCI_TRB_3_TC_BIT       (1U << 1)   /* command ring only */
#define XHCI_TRB_3_ENT_BIT      (1U << 1)   /* transfer ring only */
#define XHCI_TRB_3_ISP_BIT      (1U << 2)
#define XHCI_TRB_3_NSNOOP_BIT       (1U << 3)
#define XHCI_TRB_3_CHAIN_BIT        (1U << 4)
#define XHCI_TRB_3_IOC_BIT      (1U << 5)
#define XHCI_TRB_3_IDT_BIT      (1U << 6)
#define XHCI_TRB_3_TBC_GET(x)       (((x) >> 7) & 3)
#define XHCI_TRB_3_TBC_SET(x)       (((x) & 3) << 7)
#define XHCI_TRB_3_BEI_BIT      (1U << 9)
#define XHCI_TRB_3_DCEP_BIT     (1U << 9)
#define XHCI_TRB_3_PRSV_BIT     (1U << 9)
#define XHCI_TRB_3_BSR_BIT      (1U << 9)
#define XHCI_TRB_3_TRT_MASK     (3U << 16)
#define XHCI_TRB_3_TRT_NONE     (0U << 16)
#define XHCI_TRB_3_TRT_OUT      (2U << 16)
#define XHCI_TRB_3_TRT_IN       (3U << 16)
#define XHCI_TRB_3_DIR_IN       (1U << 16)
#define XHCI_TRB_3_TLBPC_GET(x)     (((x) >> 16) & 0xF)
#define XHCI_TRB_3_TLBPC_SET(x)     (((x) & 0xF) << 16)
#define XHCI_TRB_3_EP_GET(x)        (((x) >> 16) & 0x1F)
#define XHCI_TRB_3_EP_SET(x)        (((x) & 0x1F) << 16)
#define XHCI_TRB_3_FRID_GET(x)      (((x) >> 20) & 0x7FF)
#define XHCI_TRB_3_FRID_SET(x)      (((x) & 0x7FF) << 20)
#define XHCI_TRB_3_ISO_SIA_BIT      (1U << 31)
#define XHCI_TRB_3_SUSP_EP_BIT      (1U << 23)
#define XHCI_TRB_3_SLOT_GET(x)      (((x) >> 24) & 0xFF)
#define XHCI_TRB_3_SLOT_SET(x)      (((x) & 0xFF) << 24)

/* Commands */
#define XHCI_TRB_TYPE_RESERVED      0x00
#define XHCI_TRB_TYPE_NORMAL        0x01
#define XHCI_TRB_TYPE_SETUP_STAGE   0x02
#define XHCI_TRB_TYPE_DATA_STAGE    0x03
#define XHCI_TRB_TYPE_STATUS_STAGE  0x04
#define XHCI_TRB_TYPE_ISOCH     0x05
#define XHCI_TRB_TYPE_LINK      0x06
#define XHCI_TRB_TYPE_EVENT_DATA    0x07
#define XHCI_TRB_TYPE_NOOP      0x08
#define XHCI_TRB_TYPE_ENABLE_SLOT   0x09
#define XHCI_TRB_TYPE_DISABLE_SLOT  0x0A
#define XHCI_TRB_TYPE_ADDRESS_DEVICE    0x0B
#define XHCI_TRB_TYPE_CONFIGURE_EP  0x0C
#define XHCI_TRB_TYPE_EVALUATE_CTX  0x0D
#define XHCI_TRB_TYPE_RESET_EP      0x0E
#define XHCI_TRB_TYPE_STOP_EP       0x0F
#define XHCI_TRB_TYPE_SET_TR_DEQUEUE    0x10
#define XHCI_TRB_TYPE_RESET_DEVICE  0x11
#define XHCI_TRB_TYPE_FORCE_EVENT   0x12
#define XHCI_TRB_TYPE_NEGOTIATE_BW  0x13
#define XHCI_TRB_TYPE_SET_LATENCY_TOL   0x14
#define XHCI_TRB_TYPE_GET_PORT_BW   0x15
#define XHCI_TRB_TYPE_FORCE_HEADER  0x16
#define XHCI_TRB_TYPE_NOOP_CMD      0x17

/* Events */
#define XHCI_TRB_EVENT_TRANSFER     0x20
#define XHCI_TRB_EVENT_CMD_COMPLETE 0x21
#define XHCI_TRB_EVENT_PORT_STS_CHANGE  0x22
#define XHCI_TRB_EVENT_BW_REQUEST       0x23
#define XHCI_TRB_EVENT_DOORBELL     0x24
#define XHCI_TRB_EVENT_HOST_CTRL    0x25
#define XHCI_TRB_EVENT_DEVICE_NOTIFY    0x26
#define XHCI_TRB_EVENT_MFINDEX_WRAP 0x27

/* Error codes */
#define XHCI_TRB_ERROR_INVALID      0x00
#define XHCI_TRB_ERROR_SUCCESS      0x01
#define XHCI_TRB_ERROR_DATA_BUF     0x02
#define XHCI_TRB_ERROR_BABBLE       0x03
#define XHCI_TRB_ERROR_XACT     0x04
#define XHCI_TRB_ERROR_TRB      0x05
#define XHCI_TRB_ERROR_STALL        0x06
#define XHCI_TRB_ERROR_RESOURCE     0x07
#define XHCI_TRB_ERROR_BANDWIDTH    0x08
#define XHCI_TRB_ERROR_NO_SLOTS     0x09
#define XHCI_TRB_ERROR_STREAM_TYPE  0x0A
#define XHCI_TRB_ERROR_SLOT_NOT_ON  0x0B
#define XHCI_TRB_ERROR_ENDP_NOT_ON  0x0C
#define XHCI_TRB_ERROR_SHORT_PKT    0x0D
#define XHCI_TRB_ERROR_RING_UNDERRUN    0x0E
#define XHCI_TRB_ERROR_RING_OVERRUN 0x0F
#define XHCI_TRB_ERROR_VF_RING_FULL 0x10
#define XHCI_TRB_ERROR_PARAMETER    0x11
#define XHCI_TRB_ERROR_BW_OVERRUN   0x12
#define XHCI_TRB_ERROR_CONTEXT_STATE    0x13
#define XHCI_TRB_ERROR_NO_PING_RESP 0x14
#define XHCI_TRB_ERROR_EV_RING_FULL 0x15
#define XHCI_TRB_ERROR_INCOMPAT_DEV 0x16
#define XHCI_TRB_ERROR_MISSED_SERVICE   0x17
#define XHCI_TRB_ERROR_CMD_RING_STOP    0x18
#define XHCI_TRB_ERROR_CMD_ABORTED  0x19
#define XHCI_TRB_ERROR_STOPPED      0x1A
#define XHCI_TRB_ERROR_LENGTH       0x1B
#define XHCI_TRB_ERROR_BAD_MELAT    0x1D
#define XHCI_TRB_ERROR_ISOC_OVERRUN 0x1F
#define XHCI_TRB_ERROR_EVENT_LOST   0x20
#define XHCI_TRB_ERROR_UNDEFINED    0x21
#define XHCI_TRB_ERROR_INVALID_SID  0x22
#define XHCI_TRB_ERROR_SEC_BW       0x23
#define XHCI_TRB_ERROR_SPLIT_XACT   0x24
};

struct Td {
    int id;
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
    
    /* word 5-7 */
    uint rsvd[3];
}; 


struct Packed32B {
    uint words[8]; 
};

struct Packed16B {
    uint words[4]; 
};

/* typedefs */
typedef struct Ctlr Ctlr; 
typedef struct Trb Trb; 
typedef struct Td Td; 
typedef struct SlotCtx slotCtx; 
typedef struct EpCtx epCtx; 
typedef struct Packed32B packed32B; 
typedef struct Packed16B packed16B; 

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
    // needed for > 1 xhc's but might be too slow
    //uint p; 
    //switch what {
    //  case XHCI_CAPA: 
    //    p = ctlr->port; 
    //    break;
    //  case XHCI_CAPA: 
    //    p = ctlr->port; 
    //    break; 
    //  default: 
    //    print("write not implemented")
    //    p = ctlr->port; 

    uint read = INL(offset); 
    print("writing %#ux to offset %#ux", ((read & ~mask) | new), offset);
    OUTL(offset, ((read & ~mask) | new));
}

static uint
xhcireg_rd(Ctlr *ctlr, uint offset, uint mask) 
{
    return (INL(offset) & mask); 
}

static Ctlr* ctlrs[Nhcis];

/**********************************************
 * Top level interface functions -- debugging * 
 **********************************************/
static char*
seprintep(char *s, char *e, Ep *ep)
{
//    Ctlio *cio;
//    Qio *io;
//    Isoio *iso;
//    Ctlr *ctlr;
//
//    ctlr = ep->hp->aux;
//    ilock(ctlr);
//    
//    if(ep->aux == nil){
//        *s = 0;
//        iunlock(ctlr);
//        return s;
//    }
//    
//    switch(ep->ttype){
//        case Tctl:
//            cio = ep->aux;
//            s = seprint(s,e,"cio %#p qh %#p id %#x tog %#x tok %#x err %s\n", cio, cio->qh, cio->usbid, cio->toggle, cio->tok, cio->err);
//            break;
//        case Tbulk:
//        case Tintr:
//            io = ep->aux;
//            if(ep->mode != OWRITE)
//                s = seprint(s,e,"r: qh %#p id %#x tog %#x tok %#x err %s\n", io[OREAD].qh, io[OREAD].usbid, io[OREAD].toggle, io[OREAD].tok, io[OREAD].err);
//            if(ep->mode != OREAD)
//                s = seprint(s,e,"w: qh %#p id %#x tog %#x tok %#x err %s\n",io[OWRITE].qh, io[OWRITE].usbid, io[OWRITE].toggle, io[OWRITE].tok, io[OWRITE].err);
//            break;
//        case Tiso:
//            iso = ep->aux;
//            s = seprint(s,e,"iso %#p id %#x tok %#x tdu %#p tdi %#p err %s\n", iso, iso->usbid, iso->tok, iso->tdu, iso->tdi, iso->err);
//            break;
//    }
//    
//    iunlock(ctlr);
//    return s;
}

static void
dump(Hci *hp)
{
}

#define MIN(x, y) ((x) > (y)) ? (y) : (x)

static void 
printmem(uint start, uint size) {
    int arr = (int) start;
    print("Memory Dump starting at 0x%#ux, length 0x%#ux words\n", start, size);
    for(uint i=0; i<size; i++) {
        print("mem[%p]: 0x%#ux\n", (int*)arr, (uint)inl(arr));
        arr+=4;
    }
}



/**************************************************
 * Top level interface functions -- functionality * 
 **************************************************/
static void
init(Hci *hp)
{
    dprint("xhci init\n");
    return; 
}

static int
portreset(Hci *hp, int port, int on)
{
    dprint("xhci portreset\n");
    return -1; 
}

static int
portenable(Hci *hp, int port, int on)
{
    dprint("xhci portenable\n");
    return -1; 
}

static int
portstatus(Hci *hp, int port)
{
    dprint("xhci portstatus\n");
    return -1;
}

static void
epclose(Ep *ep)
{
    dprint("xhci epclose\n");
    return; 
}

static void
epopen(Ep *ep)
{
    dprint("xhci epopen\n");
    return; 
}

static long
epwrite(Ep *ep, void *a, long count)
{
    dprint("xhci epwrite\n");
    return -1;
}

static long
epread(Ep *ep, void *a, long count)
{
    dprint("xhci epread\n");
    return -1;
}

static void
interrupt(Ureg*, void *a)
{
    dprint("xhci interrupt\n");
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
    int io;
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
         * Find UHCI controllers (Programming Interface = 0).
         */

        /*
         * XHCI controllers programming interface = 0x30
         */

        // what are these things? ccrb/ccru? 
        // we are looking for serial and usb -- still true for xhci?
        // Found this online: 
        // ccrb -- (base class code) controller types
        // ccru -- (sub-class code)
        // ccrp -- programming interface class mode?? Not sure..
        if(p->ccrb != Pcibcserial || p->ccru != Pciscusb || p->ccrp != 0x30)
            continue;

        // map registers into memory
        // TODO not sure about the BAR values, might have to try different things
        io = p->mem[XHCI_PCI_BAR].bar & ~0x0F;

        if(io == 0){
            print("xhci: %#x %#x: failed to map registers\n", p->vid, p->did);
            continue;
        } else {
            print("xhci: vid:%#x did:%#x: successfully mapped registers at %d\n", p->vid, p->did, io);
        }
  
        // ioalloc(int port, int size, int align, char *tag) -- actually allocate this memory on host side
        if(ioalloc(io, p->mem[XHCI_PCI_BAR].size, 0, "usbxhci") < 0){
            print("usbxhci: port %#ux in use\n", io);
            continue;
        }
  
        if(p->intl == 0xFF || p->intl == 0){
            print("usbxhci: no irq assigned for port %#ux\n", io);
            continue;
        }

        print("xhci: %#x %#x: port %#ux size %#x irq %d\n", p->vid, p->did, io, p->mem[XHCI_PCI_BAR].size, p->intl);

        ctlr = malloc(sizeof(Ctlr));
        if (ctlr == nil)
            panic("xhci: out of memory");
  
        ctlr->pcidev = p;
        ctlr->port = io;
 
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

static void
xhcimeminit(Ctlr *ctlr)
{
    // allocate memory for slot (context + MaxSlotsEn) * Packed32B
    slotCtx *slot_ctx = xspanalloc(sizeof(struct SlotCtx), _64B, _64B);
    packed32B *packed_slot = xspanalloc(sizeof(struct Packed32B), _64B, _64B);
    
    // for ep contexts
    //uint num_ep = XHCI_MAXSLOTSEN * 2; // FIXME == 4 for now
    uint num_ep = 4;
    epCtx *ep_ctx[5];  // +1 for ep0 base dir
    packed32B *packed_ep[5]; 
    for(int i = 0; i < 5; i++) {
        ep_ctx[i] = (epCtx *)xspanalloc(sizeof(struct EpCtx), _64B, _64B);
        packed_ep[i] = (packed32B *)xspanalloc(sizeof(struct Packed32B), _64B, _64B);
    }
    
    // configure and pack slot context
    setup_default_slot_ctx(slot_ctx); 
    pack_slot_ctx(packed_slot, slot_ctx);
    

    // configure and pack ep context
    setup_default_ep_ctx(ep_ctx[0]); 
    // mark ep in/out 
    pack_ep_ctx(packed_ep[0], ep_ctx[0]);

    // ep1 OUT
    setup_default_ep_ctx(ep_ctx[1]); 
    pack_ep_ctx(packed_ep[1], ep_ctx[1]);
    
    // ep1 IN
    setup_default_ep_ctx(ep_ctx[1]); 
    pack_ep_ctx(packed_ep[1], ep_ctx[1]);
    
    // ep2 OUT
    setup_default_ep_ctx(ep_ctx[1]); 
    pack_ep_ctx(packed_ep[1], ep_ctx[1]);
    
    // ep2 IN
    setup_default_ep_ctx(ep_ctx[1]); 
    pack_ep_ctx(packed_ep[1], ep_ctx[1]);

    // allocate the DCBAAP
    packed32B **dcbaap = (packed32B **)xspanalloc((sizeof(void *) * (1+XHCI_MAXSLOTSEN) * 2), _64B, _64B); 
    dcbaap[0] = packed_slot;
    dcbaap[1] = packed_ep[0];
    dcbaap[2] = packed_ep[1];
    dcbaap[3] = packed_ep[2];
    dcbaap[4] = packed_ep[3];
    dcbaap[5] = packed_ep[4];
    ctlr->devctx_bar = ((uint)dcbaap & 0xFFFFFFFF); 
    
    
    // allocate the first command TRB TODO
    ctlr->cmd_ring_bar = 0; 
}
    
static void
xhcireset(Ctlr *ctlr)
{
    int i; 
    // TODO why do I need this lock? 
    ilock(ctlr);
    
    print("xhci %#ux reset\n", ctlr->port);
    
    // do I need to do this? 
    xhcireg_wr(ctlr, USBCMD_OFF, USBCMD_RESET, USBCMD_RESET_RESET);/* global reset */
    
    i = 0; 
    while (xhcireg_rd(ctlr, USBSTS_OFF, USBSTS_CNR) != USBSTS_CNR_READY) {
        // WAIT until timeout
        delay(1);
        if ((i = i + 1) == 100) {
            print("xhci controller reset timed out, USBSTS_CNR = %d\n", xhcireg_rd(ctlr, USBSTS_OFF, USBSTS_CNR));
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
    for (i = 0; i < ctlr->num_port; i++) {
        if (xhcireg_rd(ctlr, (PORTSC_OFF+i*PORTSC_ENUM_OFF), PORTSC_CCS) == PORTSC_CCS_CONNECT) {
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
        if(hp->port == 0 || hp->port == ctlr->port){
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
    hp->port = ctlr->port;
    hp->irq = p->intl;
    hp->tbdf = p->tbdf;
    // TODO change this?
    hp->nports = 2;/* default */

    // read some stuff and set global values
    // FIXME not work with > 1 controller
    caplength = xhcireg_rd(ctlr, CAPLENGTH_OFF, CAPLENGTH);
    ctlr->caplength = caplength; 
    ctlr->num_port = xhcireg_rd(ctlr, HCSPARAMS1_OFF, HCSPARAMS1_MAXPORT) >> 24;
    ctlr->oper = (uint)ctlr->port + caplength + 1; 

    print("usbxhci: caplength %d num_port %d\n", caplength, ctlr->num_port);
    print("CAP base 0x%#xx OPER base 0x%#ux\n", ctlr->port, ctlr->oper);
    
    // print all the capability registers
    printmem(ctlr->port, (0x20/4)); 

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
    print("readback: MAX_SLOT_EN: %d should be 2", xhcireg_rd(ctlr, CONFIG_OFF, CONFIG_MAXSLOTEN));

    // DCBAAP_LO = ctlr->devctx_bar
    xhcireg_wr(ctlr, DCBAAP_OFF, DCBAAP_LO, ctlr->devctx_bar);
    print("readback: DCBAAP_LO: 0x%#ux should be 0x%#ux", xhcireg_rd(ctlr, DCBAAP_OFF, DCBAAP_LO), ctlr->devctx_bar);
    
    // DCBAAP_HI = 0
    xhcireg_wr(ctlr, (DCBAAP_OFF + DCBAAP_HI_OFF), DCBAAP_HI, ZERO);
    print("readback: DCBAAP_HI: 0x%#ux should be 0", xhcireg_rd(ctlr, (DCBAAP_OFF+DCBAAP_HI_OFF), DCBAAP_HI));
    
    // CRCR_CMDRING_LO = ctlr->cmd_ring_bar
    xhcireg_wr(ctlr, CRCR_OFF, CRCR_CMDRING_LO, ctlr->cmd_ring_bar);
    print("TODO: not reading back cmdring addr for now cuz they are 0's\n");

    // CRCR_CMDRING_HI = 0
    xhcireg_wr(ctlr, (CRCR_OFF + CRCR_CMDRING_HI_OFF), CRCR_CMDRING_HI, ZERO);

    // tell the controller to run
    xhcireg_wr(ctlr, USBCMD_OFF, USBCMD_RS, USBCMD_RS_RUN);
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

    // test the controller is alive and running by reading some values
    //while(1) {
    //    delay(10);
    //    int new;
    //    if ((new = port_new_attach(ctlr)) != -1) {
    //        print("new device attached at %d\n", new);
    //    }
    //}
    // TODO remove the test code


    return 0;
}

void
usbxhcilink(void)
{
    addhcitype("xhci", reset);
}
