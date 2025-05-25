#NoEnv  ;Aploium
#SingleInstance Force
;#Warn
SendMode Input
SetWorkingDir %A_ScriptDir%
SetBatchLines -1

str1=curl "http://10.202.81.91/wescms/handler.php?catalog_id=6&shijuanbh=
str2=&cmd=kaoshi_shijuan_edit&page=0" -H "Host: 10.202.81.91" -H "User-Agent: Mozilla/5.0 (Windows NT 10.0; Win64; x64; rv:42.0) Gecko/20100101 Firefox/42.0" -H "Accept: text/html,application/xhtml+xml,application/xml;q=0.9,*/*;q=0.8" -H "Accept-Language: zh-CN,zh;q=0.8,en-US;q=0.5,en;q=0.3" --compressed -H "DNT: 1" -H "Referer: http://10.202.81.91/wescms/handler.php?catalog_id=6&cmd=kaoshi_shijuan_list&kaoshih=182128" -H "Cookie: wes_w=1920; wes_h=1080; PHPSESSID=c2r8c5r4um425jutjrr2ah0r41" -H "X-Forwarded-For: 127.0.0.1" -H "X-originating-IP: 127.0.0.1" -H "X-remote-addr: 127.0.0.1" -H "Client_ip: 127.0.0.1" -H "Connection: keep-alive" -H "Pragma: no-cache" -H "Cache-Control: no-cache" -o 

i := 182130
while(i<182138)
	Run,%str1%%i%%str2%%i%.htm




ExitApp
;NumLock & -::Reload
;NumLock & =::ExitApp
