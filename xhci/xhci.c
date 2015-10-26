/* 
 * USB Extensible Host Controller Interface (xHCI) driver
 * SuperSpeed USB 3.0
 *
 *
 * BUGS:
 * - Not Implement
 */


#include	"u.h"
#include	"../port/lib.h"
#include	"mem.h"
#include	"dat.h"
#include	"fns.h"
#include	"io.h"
#include	"../port/error.h"
#include	"../port/usb.h"

#define diprint		if(xhcidebug || iso->debug)print
#define ddiprint	if(xhcidebug>1 || iso->debug>1)print
#define dqprint		if(xhcidebug || (qh->io && qh->io->debug))print
#define ddqprint	if(xhcidebug>1 || (qh->io && qh->io->debug>1))print

#define TRUNC(x, sz)	((x) & ((sz)-1))
#define LPTR(q)		((ulong*)KADDR((q) & ~0x1F))

typedef struct Ctlio Ctlio;
typedef union Ed Ed;
typedef struct Edpool Edpool;
typedef struct Itd Itd;
typedef struct Qio Qio;
typedef struct Qtd Qtd;
typedef struct Sitd Sitd;
typedef struct Td Td;

