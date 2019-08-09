#ifndef DDK750_HELP_H__
#define DDK750_HELP_H__
#include "ddk750_chip.h"
#ifndef USE_INTERNAL_REGISTER_ACCESS

#include <linux/ioport.h>
#include <asm/io.h>
#include <asm/uaccess.h>
//#include "../lynx_help.h"
#include "ddk750_mode.h"


#define PEEK32(addr) readl((addr)+mmio750)
#define POKE32(addr,data) writel((data),(addr)+mmio750)
#define peekRegisterDWord PEEK32
#define pokeRegisterDWord POKE32


#define peekRegisterByte(addr) readb((addr)+mmio750)
#define pokeRegisterByte(addr,data) writeb((data),(addr)+mmio750)



extern volatile unsigned  char __iomem * mmio750;
extern char revId750;
extern unsigned short devId750;
void ddk750_set_mmio(volatile unsigned char *,unsigned short,char);


#else
/* implement if you want use it*/
#endif

#endif

