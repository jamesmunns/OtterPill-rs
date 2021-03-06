MEMORY
{
  /* NOTE K = KiBi = 1024 bytes */
  FLASH  : ORIGIN = 0x08000000, LENGTH = 64K
  RAM    : ORIGIN = 0x20000000, LENGTH = 15K
  PANDUMP: ORIGIN = 0x20003C00, LENGTH = 1K
}

_panic_dump_start = ORIGIN(PANDUMP);
_panic_dump_end   = ORIGIN(PANDUMP) + LENGTH(PANDUMP);
