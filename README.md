# Music-player
Hello guys how’s it going  ?.
 today we have also another project base on USB protocol but this time we act as host because the hierarchal of USB this make it able to support a verity types of devices and communication types 
So without any future ado let’s cut to the chase .
First of all the main idea behind this project is that we read a specific directory in the flash memory and collect all supported files currently in our case is WVA file format and at the same time extract the important information from that header and store all the information in struct then we start music player functionality so first set the I2S peripheral with required frequency then open the file and read samples and send these samples to I2S peripheral through DMA to make my project more flexible and useful I put some of human interfaces to control the flow of the running audio like (Next – Prev – pause )
Also there is mute key to mute the output signal 
I hope this brief introduction about this project help you to understand my project architecture and also help you if you will try to build your own enhanced  version from this project . I hope the next version from this project will be introduced to you soon . 


youtube video :
https://www.youtube.com/watch?v=yJaLsAO2xxQ
