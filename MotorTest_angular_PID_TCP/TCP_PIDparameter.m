% % 創建Serial物件
% s = serial('COM3', 'BaudRate', 115200);
% fopen(s);
% 
% % 寫入數據
% fwrite(s, 'a');
% 
% % 關閉串口
% fclose(s);
% -----------------------------------------------
% 建立 TCP/IP 客戶端，連接到 ESP32
% t = tcpclient('192.168.125.107', 80);
% t = tcpclient('192.168.91.112', 80);
% t = tcpclient('192.168.1.201', 80);
% t = tcpclient('172.20.10.13', 80);
% t = tcpclient('172.20.10.13', 80);
clc;clear;
t = tcpclient('192.168.65.112', 80);
% t = tcpclient('192.168.65.55', 80);
fopen(t);
% 傳送字串 'Hello ESP32!'
% write(t, 'Hello ESP32!');
% string PID {rollP,rollI,rollD,pitchP,pitchI,pitchD,yawP,yawI,yawD}
% fwrite(t, '0.5,0.000001,1.2,0.5,0.0,1.5,0.5,0.0,0.7');
% fwrite(t, '6,1,1.5,6,1,1.5,6,2,2');
% fwrite(t, '7,2,2,7,2,2,10,1,1'); % dsliding mode
% fwrite(t, '20,8,5,7,2,2,10,2,2'); % dsliding mode
fwrite(t, '7,2,1.5,6,1,1.5,7,1,1'); % sliding mode
% fwrite(t, '0.4,0.0,1.1,0.5,0.0,1.1,0.5,0.0,0.6'); % PID mode
% write(t, '1.5,2.0,1.5,5.5,4.0,4.5,4.7,0.0,22.7');
% 關閉 TCP/IP 客戶端
% delete(t);
% fclose(t);

