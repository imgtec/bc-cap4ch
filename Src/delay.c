void udelay(unsigned int usec)
{
        unsigned int kv;

        do {
		kv = 5;
		while(kv--);
        } while(usec--);
}

void mdelay(unsigned int msec)
{
        while (msec--)
                udelay(1000);
}
