ip1 = '192.168.56.1';
port1 = 3322;
ip2 = '198.21.207.186';
port2 = 3323;
udp1 = udp(ip2,port2,'LocalPort',port1);
fopen(udp1)