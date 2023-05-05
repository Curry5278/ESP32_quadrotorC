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
t = tcpclient('192.168.125.107', 80);
% 傳送字串 'Hello ESP32!'
% write(t, 'Hello ESP32!');
% string PID {rollP,rollI,rollD,pitchP,pitchI,pitchD,yawP,yawI,yawD}
write(t, '0.5,0.0,1.5,0.5,0.0,1.5,0.7,0.0,0.7');
% write(t, '1.5,2.0,1.5,5.5,4.0,4.5,4.7,0.0,22.7');
% 關閉 TCP/IP 客戶端
delete(t);

