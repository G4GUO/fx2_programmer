INCLUDES = -I../include
CC = sdcc -mmcs51
BASENAME   = flash_loader

$(BASENAME).ihx: $(BASENAME).c
	$(CC) $(INCLUDES) -DUSB16 $(BASENAME).c  -o$(BASENAME).ihx

clean:
	rm -f $(BASENAME).asm
	rm -f $(BASENAME).ihx
	rm -f $(BASENAME).lnk
	rm -f $(BASENAME).lst
	rm -f $(BASENAME).map
	rm -f $(BASENAME).mem
	rm -f $(BASENAME).rel
	rm -f $(BASENAME).rst
	rm -f $(BASENAME).sym
	

