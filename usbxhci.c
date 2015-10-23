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
#include"../port/usb.h"

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

/* Hard coded values */
#define XHCI_PCI_BAR 0
#define XHCI_MAXSLOTSEN 2
#define _64B 64

/*************************************************
 * Controls for reading/writing of the registers *
 *************************************************/
/* register offsets (from base address of Capability Register) */
/* operational register */
// this definitely does not work if we have > 1 controller
#define OPREG_OFF caplength // TODO will this work?
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
#define USBCMD_RESET_RESET 1 
#define PORTSC_CCS_CONNECT 1

/* capability register */




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

}; 

struct Td {

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
static inline void
pack_slot_ctx(packed32B *packed, slotCtx *slot) {
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
    
    uint word4 = (slots->maxEsitLo & 0xFF) << 24;
    word4 = word4 | (slots->devAddr & 0xFFFF);
    
    packed->words[0] = word0;
    packed->words[1] = word1;
    packed->words[2] = word2;
    packed->words[3] = word3;
    packed->words[4] = word4;
    return; 
}


/** @brief This function will write to one of the registers
 *  @param TODO
 **/
static void
xhcireg_wr(Ctlr *ctlr, uint offset, uint mask, uint new) 
{
    uint read = INL(offset); 
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
    Ctlio *cio;
    Qio *io;
    Isoio *iso;
    Ctlr *ctlr;

    ctlr = ep->hp->aux;
    ilock(ctlr);
    
    if(ep->aux == nil){
        *s = 0;
        iunlock(ctlr);
        return s;
    }
    
    switch(ep->ttype){
        case Tctl:
            cio = ep->aux;
            s = seprint(s,e,"cio %#p qh %#p id %#x tog %#x tok %#x err %s\n", cio, cio->qh, cio->usbid, cio->toggle, cio->tok, cio->err);
            break;
        case Tbulk:
        case Tintr:
            io = ep->aux;
            if(ep->mode != OWRITE)
                s = seprint(s,e,"r: qh %#p id %#x tog %#x tok %#x err %s\n", io[OREAD].qh, io[OREAD].usbid, io[OREAD].toggle, io[OREAD].tok, io[OREAD].err);
            if(ep->mode != OREAD)
                s = seprint(s,e,"w: qh %#p id %#x tog %#x tok %#x err %s\n",io[OWRITE].qh, io[OWRITE].usbid, io[OWRITE].toggle, io[OWRITE].tok, io[OWRITE].err);
            break;
        case Tiso:
            iso = ep->aux;
            s = seprint(s,e,"iso %#p id %#x tok %#x tdu %#p tdi %#p err %s\n", iso, iso->usbid, iso->tok, iso->tdu, iso->tdi, iso->err);
            break;
    }
    
    iunlock(ctlr);
    return s;
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
            print("xhci: %#x %#x: successfully mapped registers\n", p->vid, p->did);
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

        dprint("xhci: %#x %#x: port %#ux size %#x irq %d\n", p->vid, p->did, io, p->mem[4].size, p->intl);

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
uhcimeminit(Ctlr *ctlr)
{
    // TODO
    // allocate memory for slot (context + MaxSlotsEn) * Packed32B
    slotCtx *slot_ctx = xspanalloc(sizeof(struct SlotCtx), _64B, _64B);
    packed32B *packed_slot = xspanalloc(sizeof(struct Packed32B), _64B, _64B);
    
    epCtx *ep_ctx[XHCI_MAXSLOTSEN]; 
    packed32B *packed_ep[XHCI_MAXSLOTSEN]; 
    for(int i = 0; i < XHCI_MAXSLOTSEN; i++) {
        ep_ctx[i] = (epCtx *)xspanalloc(sizeof(struct EpCtx), _64B, _64B);
        packed_ep[i] = (packed32B *)xspanalloc(sizeof(struct Packed32B), _64B, _64B);
    }
    
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
    pack_slot_ctx(packed_slot, slot_ctx);
    
    // now manually configure 2 ep
    packed_ep[0]->epstate       = 0;
    packed_ep[0]->hostInitDis   = 0;
    packed_ep[0]->trDeqPtrLo    = 0;
    packed_ep[0]->trDeqPtrHi    = 0;
    pack_ep_ctx(packed_ep[0], ep_ctx[0]);
    
    packed_ep[1]->epstate       = 0;
    packed_ep[1]->hostInitDis   = 0;
    packed_ep[1]->trDeqPtrLo    = 0;
    packed_ep[1]->trDeqPtrHi    = 0;
    pack_ep_ctx(packed_ep[1], ep_ctx[1]);

    // allocate the DCBAAP
    uint *dcbaap = (uint *)xspanalloc((sizeof(void *) * (1+XHCI_MAXSLOTSEN)), _64B, _64B); 
    dcbaap[0] = packed_slot;
    dcbaap[1] = &(packed_ep[0]);
    dcbaap[2] = &(packed_ep[1]);
    ctlr->devctx_bar = dxbaap; 
    
    
    // allocate the first command TRB TODO
    ctlr->cmd_ring_bar = 0; 
}
    
static void
xhcireset(Ctlr *ctlr)
{
    int i; 
    // TODO why do I need this lock? 
    ilock(ctlr);
    dprint("xhci %#ux reset\n", ctlr->port);
    
    // do I need to do this? 
    xhcireg_wr(ctlr, USBCMD_OFF, USBCMD_RESET, USBCMD_RESET_RESET);/* global reset */
    
    i = 0; 
    while (xhcireg_rd(ctlr, USBSTS_OFF, USBSTS_CNR) != USBSTS_CNR_READY) {
        // WAIT until timeout
        delay(1);
        if ((i = i + 1) == 100) {
            print("xhci controller reset timed out\n");
            break; 
        }
    }

    // read some stuff and set global values
    // FIXME not work with > 1 controller
    caplength = xhcireg_rd(ctlr, CAPLENGTH_OFF, CAPLENGTH);
    ctlr->caplength = caplength; 
    ctlr->num_port = xhcireg_rd(ctlr, HCSPARAMS1_OFF, HCSPARAMS1_MAXPORT);
    
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

    // this call resets the chip and wait until regs are writable
    uhcireset(ctlr);
    // this call initializes data structures
    uhcimeminit(ctlr);

    // now write all the registers
    // MAX_SLOT_EN == 2
    xhcireg_wr(ctlr, CONFIG_OFF, CONFIG_MAXSLOTEN, 2);
    // DCBAAP_LO = ctlr->devctx_bar
    xhcireg_wr(ctlr, DCBAAP_OFF, DCBAAP_LO, ctlr->devctx_bar);
    // DCBAAP_HI = 0
    xhcireg_wr(ctlr, (DCBAAP_OFF + DCBAAP_HI_OFF), DCBAAP_HI, ZERO);
    // CRCR_CMDRING_LO = ctlr->cmd_ring_bar
    xhcireg_wr(ctlr, CRCR_OFF, CRCR_CMDRING_LO, ctlr->cmd_ring_bar);
    // CRCR_CMDRING_HI = 0
    xhcireg_wr(ctlr, (CRCR_OFF + CRCR_CMDRING_HI_OFF), CRCR_CMDRING_HI, ZERO);

    // tell the controller to run
    xhcireg_wr(ctlr, USBCMD_OFF, USBCMD_RS, USBCMD_RS_RUN);
    /*
     * Linkage to the generic HCI driver.
     */
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
    while(1) {
        delay(10);
        int new;
        if ((new = port_new_attach(ctlr)) != -1) {
            print("new device attached at %d\n", new);
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
