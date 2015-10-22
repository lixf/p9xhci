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

#define INB(x) inb(ctlr->port+(x))
#define INS(x) ins(ctlr->port+(x))
#define INL(x) inl(ctlr->port+(x))
#define OUTB(x, v) outb(ctlr->port+(x), (v))
#define OUTS(x, v) outs(ctlr->port+(x), (v))
#define OUTL(x, v) outl(ctlr->port+(x), (v))
#define TRUNC(x, sz) ((x) & ((sz)-1))
#define PTR(q) ((void*) KADDR((ulong)(q) & ~ (0xF|PCIWINDOW)))
#define QPTR(q) ((Qh*)PTR(q))
#define TPTR(q) ((Td*)PTR(q))
#define PORT(p) (Portsc0 + 2*(p))
#define diprint if(debug || iso->debug) print
#define ddiprint if(debug>1 || iso->debug>1) print
#define dqprint if(debug || (qh->io && qh->io->debug)) print
#define ddqprint if(debug>1 || (qh->io && qh->io->debug>1)) print

/* static values -- read from CAPREG*/
static uint caplength; 

/* Hard coded values */
#define XHCI_PCI_BAR 0

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
struct Trb; 
struct Td; 


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

/* typedefs */
typedef struct Ctlr Ctlr; 
typedef struct Trb Trb; 
typedef struct Td Td; 


/*********************************************
 * xHCI specific functions
 *********************************************/
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

/*******************************************
 * Old code starts here
 *******************************************/

static Ctlr* ctlrs[Nhcis];

static Tdpool tdpool;
static Qhpool qhpool;
static int debug;

static char* qhsname[] = { "idle", "install", "run", "done", "close", "FREE" };

static void
uhcicmd(Ctlr *ctlr, int c)
{
OUTS(Cmd, c);
}

static void
uhcirun(Ctlr *ctlr, int on)
{
int i;

ddprint("uhci %#ux setting run to %d\n", ctlr->port, on);

if(on)
uhcicmd(ctlr, INS(Cmd)|Crun);
else
uhcicmd(ctlr, INS(Cmd) & ~Crun);
for(i = 0; i < 100; i++)
if(on == 0 && (INS(Status) & Shalted) != 0)
break;
else if(on != 0 && (INS(Status) & Shalted) == 0)
break;
else
delay(1);
if(i == 100)
dprint("uhci %#x run cmd timed out\n", ctlr->port);
ddprint("uhci %#ux cmd %#ux sts %#ux\n",
ctlr->port, INS(Cmd), INS(Status));
}

static int
tdlen(Td *td)
{
return (td->csw+1) & Tdlen;
}

static int
maxtdlen(Td *td)
{
return ((td->token>>21)+1) & (Tdmaxpkt-1);
}

static int
tdtok(Td *td)
{
return td->token & 0xFF;
}

static char*
seprinttd(char *s, char *se, Td *td)
{
s = seprint(s, se, "%#p link %#ulx", td, td->link);
if((td->link & Tdvf) != 0)
s = seprint(s, se, "V");
if((td->link & Tdterm) != 0)
s = seprint(s, se, "T");
if((td->link & Tdlinkqh) != 0)
s = seprint(s, se, "Q");
s = seprint(s, se, " csw %#ulx ", td->csw);
if(td->csw & Tdactive)
s = seprint(s, se, "a");
if(td->csw & Tdiso)
s = seprint(s, se, "I");
if(td->csw & Tdioc)
s = seprint(s, se, "i");
if(td->csw & Tdlow)
s = seprint(s, se, "l");
if((td->csw & (Tderr1|Tderr2)) == 0)
s = seprint(s, se, "z");
if(td->csw & Tderrors)
s = seprint(s, se, " err %#ulx", td->csw & Tderrors);
if(td->csw & Tdstalled)
s = seprint(s, se, "s");
if(td->csw & Tddberr)
s = seprint(s, se, "d");
if(td->csw & Tdbabble)
s = seprint(s, se, "b");
if(td->csw & Tdnak)
s = seprint(s, se, "n");
if(td->csw & Tdcrcto)
s = seprint(s, se, "c");
if(td->csw & Tdbitstuff)
s = seprint(s, se, "B");
s = seprint(s, se, " stslen %d", tdlen(td));

s = seprint(s, se, " token %#ulx", td->token);
if(td->token == 0)/* the BWS loopback Td, ignore rest */
return s;
s = seprint(s, se, " maxlen %d", maxtdlen(td));
if(td->token & Tddata1)
s = seprint(s, se, " d1");
else
s = seprint(s, se, " d0");
s = seprint(s, se, " id %#ulx:", (td->token>>15) & Epmax);
s = seprint(s, se, "%#ulx", (td->token>>8) & Devmax);
switch(tdtok(td)){
case Tdtokin:
s = seprint(s, se, " in");
break;
case Tdtokout:
s = seprint(s, se, " out");
break;
case Tdtoksetup:
s = seprint(s, se, " setup");
break;
default:
s = seprint(s, se, " BADPID");
}
s = seprint(s, se, "\n\t  buffer %#ulx data %#p", td->buffer, td->data);
s = seprint(s, se, " ndata %uld sbuff %#p buff %#p",
td->ndata, td->sbuff, td->buff);
if(td->ndata > 0)
s = seprintdata(s, se, td->data, td->ndata);
return s;
}

static void
isodump(Isoio *iso, int all)
{
char buf[256];
Td *td;
int i;

print("iso %#p %s state %d nframes %d"
" td0 %#p tdu %#p tdi %#p data %#p\n",
iso, iso->tok == Tdtokin ? "in" : "out",
iso->state, iso->nframes, iso->tdps[iso->td0frno],
iso->tdu, iso->tdi, iso->data);
if(iso->err != nil)
print("\terr='%s'\n", iso->err);
if(all == 0){
seprinttd(buf, buf+sizeof(buf), iso->tdu);
print("\ttdu %s\n", buf);
seprinttd(buf, buf+sizeof(buf), iso->tdi);
print("\ttdi %s\n", buf);
}else{
td = iso->tdps[iso->td0frno];
for(i = 0; i < iso->nframes; i++){
seprinttd(buf, buf+sizeof(buf), td);
if(td == iso->tdi)
print("i->");
if(td == iso->tdu)
print("u->");
print("\t%s\n", buf);
td = td->next;
}
}
}

static int
sameptr(void *p, ulong l)
{
if(l & QHterm)
return p == nil;
return PTR(l) == p;
}

static void
dumptd(Td *td, char *pref)
{
char buf[256];
char *s;
char *se;
int i;

i = 0;
se = buf+sizeof(buf);
for(; td != nil; td = td->next){
s = seprinttd(buf, se, td);
if(!sameptr(td->next, td->link))
seprint(s, se, " next %#p != link %#ulx %#p",
td->next, td->link, TPTR(td->link));
print("%std %s\n", pref, buf);
if(i++ > 20){
print("...more tds...\n");
break;
}
}
}

static void
qhdump(Qh *qh, char *pref)
{
char buf[256];
char *s;
char *se;
ulong td;
int i;

s = buf;
se = buf+sizeof(buf);
s = seprint(s, se, "%sqh %s %#p state %s link %#ulx", pref,
qh->tag, qh, qhsname[qh->state], qh->link);
if(!sameptr(qh->tds, qh->elink))
s = seprint(s, se, " [tds %#p != elink %#ulx %#p]",
qh->tds, qh->elink, TPTR(qh->elink));
if(!sameptr(qh->next, qh->link))
s = seprint(s, se, " [next %#p != link %#ulx %#p]",
qh->next, qh->link, QPTR(qh->link));
if((qh->link & Tdterm) != 0)
s = seprint(s, se, "T");
if((qh->link & Tdlinkqh) != 0)
s = seprint(s, se, "Q");
s = seprint(s, se, " elink %#ulx", qh->elink);
if((qh->elink & Tdterm) != 0)
s = seprint(s, se, "T");
if((qh->elink & Tdlinkqh) != 0)
s = seprint(s, se, "Q");
s = seprint(s, se, " io %#p", qh->io);
if(qh->io != nil && qh->io->err != nil)
seprint(s, se, " err='%s'", qh->io->err);
print("%s\n", buf);
dumptd(qh->tds, "\t");
if((qh->elink & QHterm) == 0){
print("\thw tds:");
i = 0;
for(td = qh->elink; (td & Tdterm) == 0; td = TPTR(td)->link){
print(" %#ulx", td);
if(td == TPTR(td)->link)/* BWS Td */
break;
if(i++ > 40){
print("...");
break;
}
}
print("\n");
}
}

static void
xdump(Ctlr *ctlr, int doilock)
{
Isoio *iso;
Qh *qh;
int i;

if(doilock){
if(ctlr == ctlrs[0]){
lock(&tdpool);
print("tds: alloc %d = inuse %d + free %d\n",
tdpool.nalloc, tdpool.ninuse, tdpool.nfree);
unlock(&tdpool);
lock(&qhpool);
print("qhs: alloc %d = inuse %d + free %d\n",
qhpool.nalloc, qhpool.ninuse, qhpool.nfree);
unlock(&qhpool);
}
ilock(ctlr);
}
print("uhci port %#x frames %#p nintr %d ntdintr %d",
ctlr->port, ctlr->frames, ctlr->nintr, ctlr->ntdintr);
print(" nqhintr %d nisointr %d\n", ctlr->nqhintr, ctlr->nisointr);
print("cmd %#ux sts %#ux fl %#ulx ps1 %#ux ps2 %#ux frames[0] %#ulx\n",
INS(Cmd), INS(Status),
INL(Flbaseadd), INS(PORT(0)), INS(PORT(1)),
ctlr->frames[0]);
for(iso = ctlr->iso; iso != nil; iso = iso->next)
isodump(iso, 1);
i = 0;
for(qh = ctlr->qhs; qh != nil; qh = qh->next){
qhdump(qh, "");
if(i++ > 20){
print("qhloop\n");
break;
}
}
print("\n");
if(doilock)
iunlock(ctlr);
}

static Td*
tdalloc(void)
{
int i;
Td *td;
Td *pool;

lock(&tdpool);
if(tdpool.free == nil){
ddprint("uhci: tdalloc %d Tds\n", Incr);
pool = xspanalloc(Incr*sizeof(Td), Align, 0);
if(pool == nil)
panic("tdalloc");
for(i=Incr; --i>=0;){
pool[i].next = tdpool.free;
tdpool.free = &pool[i];
}
tdpool.nalloc += Incr;
tdpool.nfree += Incr;
}
td = tdpool.free;
tdpool.free = td->next;
tdpool.ninuse++;
tdpool.nfree--;
unlock(&tdpool);

memset(td, 0, sizeof(Td));
td->link = Tdterm;
assert(((ulong)td & 0xF) == 0);
return td;
}

static void
tdfree(Td *td)
{
if(td == nil)
return;
free(td->buff);
td->buff = nil;
lock(&tdpool);
td->next = tdpool.free;
tdpool.free = td;
tdpool.ninuse--;
tdpool.nfree++;
unlock(&tdpool);
}

static void
qhlinkqh(Qh* qh, Qh* next)
{
if(next == nil)
qh->link = QHterm;
else{
next->link = qh->link;
next->next = qh->next;
qh->link = PCIWADDR(next)|QHlinkqh;
}
qh->next = next;
}

static void
qhlinktd(Qh *qh, Td *td)
{
qh->tds = td;
if(td == nil)
qh->elink = QHvf|QHterm;
else
qh->elink = PCIWADDR(td);
}

static void
tdlinktd(Td *td, Td *next)
{
td->next = next;
if(next == nil)
td->link = Tdterm;
else
td->link = PCIWADDR(next)|Tdvf;
}

static Qh*
qhalloc(Ctlr *ctlr, Qh *prev, Qio *io, char *tag)
{
int i;
Qh *qh;
Qh *pool;

lock(&qhpool);
if(qhpool.free == nil){
ddprint("uhci: qhalloc %d Qhs\n", Incr);
pool = xspanalloc(Incr*sizeof(Qh), Align, 0);
if(pool == nil)
panic("qhalloc");
for(i=Incr; --i>=0;){
pool[i].next = qhpool.free;
qhpool.free = &pool[i];
}
qhpool.nalloc += Incr;
qhpool.nfree += Incr;
}
qh = qhpool.free;
qhpool.free = qh->next;
qh->next = nil;
qh->link = QHterm;
qhpool.ninuse++;
qhpool.nfree--;
unlock(&qhpool);

qh->tds = nil;
qh->elink = QHterm;
qh->state = Qidle;
qh->io = io;
qh->tag = nil;
kstrdup(&qh->tag, tag);

if(prev != nil){
coherence();
ilock(ctlr);
qhlinkqh(prev, qh);
iunlock(ctlr);
}

assert(((ulong)qh & 0xF) == 0);
return qh;
}

static void
qhfree(Ctlr *ctlr, Qh *qh)
{
Td *td;
Td *ltd;
Qh *q;

if(qh == nil)
return;

ilock(ctlr);
for(q = ctlr->qhs; q != nil; q = q->next)
if(q->next == qh)
break;
if(q == nil)
panic("qhfree: nil q");
q->next = qh->next;
q->link = qh->link;
iunlock(ctlr);

for(td = qh->tds; td != nil; td = ltd){
ltd = td->next;
tdfree(td);
}
lock(&qhpool);
qh->state = Qfree;/* paranoia */
qh->next = qhpool.free;
qh->tag = nil;
qh->io = nil;
qhpool.free = qh;
qhpool.ninuse--;
qhpool.nfree++;
unlock(&qhpool);
ddprint("qhfree: qh %#p\n", qh);
}

static char*
errmsg(int err)
{
if(err == 0)
return "ok";
if(err & Tdcrcto)
return "crc/timeout error";
if(err & Tdbabble)
return "babble detected";
if(err & Tddberr)
return "db error";
if(err & Tdbitstuff)
return "bit stuffing error";
if(err & Tdstalled)
return Estalled;
return Eio;
}

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
    xdump(hp->aux, 1);
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
    // allocate the device context
    ctlr->devctx_bar = 0; 
    // allocate the first command TRB
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
