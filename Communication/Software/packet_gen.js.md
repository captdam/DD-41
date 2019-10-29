The purpose of this program is to help the developer calculate checksum for this project.

Execute this program in browser console to generate packet:

```javascript
var packet = '222efghijklmnopqrstuvwxyzuuuuu0000000';
for (var i = 0, sum = 0; i < 32; i++) {
  sum += packet.charCodeAt(i);
}
sum = 256 - (sum % 256);
console.log('Checksum is: '+sum+' Complete Tx packet should be: '+String.fromCharCode(sum)+packet);
```

Provide the data you want to send in variable ```packet```

The packet should be 32-byte long. (In another word, 32 ASCII characters.) The packet can be longer than 32 byte, but the hardware will ignor the extra words.

The program will return the complete packet (checksum + packet body). Just copy it and paste into serial test software and send.
