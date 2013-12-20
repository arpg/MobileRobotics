MobileRobotics
==============

2D Simulator for Mobile Robotics

Before use it, make sure you already compile pent.c with mex.

-----------------
How To Run (In three Matlab instances):

Run TestClient
 % usage:   close all; clear all; TestClient('lumaaaa','r',8888);

Server 
 % usage:   clear all; clear all;server = Server('DilatedMap.png');server.Run();

WorldProxy

 % usage:   close all; clear all;w=WorldProxy(8888,'localhost');w.Run();
