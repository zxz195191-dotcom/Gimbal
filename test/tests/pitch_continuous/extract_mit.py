"""Compile the production MIT encoder unchanged, with only the CAN sink mocked."""
import re,sys
from pathlib import Path
s=Path(sys.argv[1]).read_text(encoding='utf-8-sig')
out=['#include "headfile.h"', '#include "yaw_diagnostics.h"', 'HAL_StatusTypeDef Can_Send_Std(CAN_HandleTypeDef *, uint16_t, uint8_t *, uint8_t);']
out+=re.findall(r'^#define DM_(?:P|V|KP|KD|T)_(?:MIN|MAX)\s+[^\n]+',s,re.M)
for name in ['int float_to_uint(', 'float uint_to_float(', 'HAL_StatusTypeDef dm4310_send_mit(']:
 a=s.index(name); b=s.index('{',a); depth=1; e=b+1
 while depth:
  depth+=(s[e]=='{')-(s[e]=='}');e+=1
 out.append(s[a:e])
Path(sys.argv[2]).write_text('\n'.join(out),encoding='utf-8')
