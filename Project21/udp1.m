ipB = '192.168.56.1'; portA = 9090;
ipA = '198.21.204.174'; portB = 9091;
udpB = udp(ipA,portA,'LocalPort',portB);
fopen(udpB);

fwrite(udpB,65:74)
A = fread(udpB,10)


echoudp('off')
fclose(udpB)