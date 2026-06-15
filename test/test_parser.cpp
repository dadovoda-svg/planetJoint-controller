#include "JointBusProtocol.h"
#include <cstdio>
int main(){
  uint8_t b[]={0xA5,0x40,0x46,0x00,0x7F,0xB5,0xD1};
  JointBus::Parser p; JointBus::Frame f; JointBus::Parser::Result r=JointBus::Parser::Result::None;
  for(auto x:b) r=p.push(x,f);
  if(r!=JointBus::Parser::Result::FrameReady) return 1;
  if(f.address!=4 || f.seq!=0x46 || f.command!=JointBus::Command::Ping) return 2;
  printf("ok addr=%u seq=%u cmd=%02X\n", f.address, f.seq, (unsigned)f.command);
  return 0;
}
