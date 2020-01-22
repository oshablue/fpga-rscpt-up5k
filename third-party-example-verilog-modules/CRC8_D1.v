////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 1999-2008 Easics NV.
// This source file may be used and distributed without restriction
// provided that this copyright statement is not removed from the file
// and that any derivative work contains the original copyright notice
// and the associated disclaimer.
//
// THIS SOURCE FILE IS PROVIDED "AS IS" AND WITHOUT ANY EXPRESS
// OR IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
// WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
//
// Purpose : synthesizable CRC function
//   * polynomial: x^8 + x^2 + x^1 + 1
//   * data width: 1
//
// Info : tools@easics.be
//        http://www.easics.com
////////////////////////////////////////////////////////////////////////////////
module CRC8_D1;

  // polynomial: x^8 + x^2 + x^1 + 1
  // data width: 1
  // convention: the first serial bit is D[0]
  function [7:0] nextCRC8_D1;

    input Data;
    input [7:0] crc;
    reg [0:0] d;
    reg [7:0] c;
    reg [7:0] newcrc;
  begin
    d[0] = Data;
    c = crc;

    newcrc[0] = d[0] ^ c[7];
    newcrc[1] = d[0] ^ c[0] ^ c[7];
    newcrc[2] = d[0] ^ c[1] ^ c[7];
    newcrc[3] = c[2];
    newcrc[4] = c[3];
    newcrc[5] = c[4];
    newcrc[6] = c[5];
    newcrc[7] = c[6];
    nextCRC8_D1 = newcrc;
  end
  endfunction
endmodule
