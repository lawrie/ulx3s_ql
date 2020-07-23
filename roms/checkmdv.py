#!/usr/bin/env python3

class check_mdv:
  def __init__(self):
    self.xyz=0
    print("mdv checker")
    self.byte_buf=bytearray(1)
    self.word_buf=bytearray(2)
    self.preamble_buf=bytearray(12) # intial 0
    self.preamble_buf[10]=0xFF
    self.preamble_buf[11]=0xFF
    self.header_buf=bytearray(16)
    self.header_buf[0]=0xFF
    self.data_buf=bytearray(1024)
    self.mdv_state_sync=0
    self.mdv_state_preamble=1
    self.mdv_state_header=1
    self.mdv_state_blkid=0

  def get(self,filedata):
    mv=memoryview(self.data_buf)
    i=0
    if self.mdv_state_sync:
      print("state_sync at 0x%X" % filedata.tell())
      # searching for sync in next 1000 bytes
      j=0
      n=0
      while j<1000:
        if filedata.readinto(mv[0:2]): # read 2 bytes
          if mv[0]==0x5A and mv[1]==0x5A: #sync
            n+=1
            continue
          if mv[0]==0 and mv[1]==0 and n>10: # end of sync, start of preamble
            print("end of sync found at 0x%X" % filedata.tell())
            self.mdv_state_sync=0
            self.mdv_state_preamble=1
            #self.mdv_state_header=1
            i=2 # continue with preamble
            break
          else:
            print("unexpected data")
            print(bytearray(mv[0:2]))
            n=0
        else: # EOF, make it circular
          filedata.seek(0)
        j+=2
    if self.mdv_state_preamble:
      print("state_preamble")
      # searching for preamble in next 1000 bytes
      j=0
      while j<1000:
        if filedata.readinto(mv[i:i+2]): # read 2 bytes
          if mv[i]==0xFF and mv[i+1]==0xFF and i>=10:
            # long preamble found
            self.mdv_state_preamble=0
            #self.mdv_state_header=1
            break
          else:
            if mv[i]==0 and mv[i+1]==0:
              i+=2
            else:
              print("unexpected data at 0x%X" % filedata.tell())
              print(bytearray(mv[i:i+2]))
              i=0
          j+=2
        else: # EOF, make it circular
          filedata.seek(0)
      if self.mdv_state_preamble==0:
        print("preamble found")
        print(bytearray(mv[0:i+2]))
    else: # not pramble: header or data
      #print("state blk_id+short_preamble or header/data")
      if self.mdv_state_blkid:
        filedata.readinto(mv[0:12])
        print("blkid+short_preamble")
        print(self.data_buf[0:12])
        self.mdv_state_blkid=0
        self.mdv_state_preamble=0
      else: # header or data
        filedata.readinto(mv[0:16])
        #if mv[0]==0xFF:
        if self.mdv_state_header:
          print("header at 0x%X" % filedata.tell())
          print(self.data_buf[0:16])
          self.mdv_state_blkid=1
          # next time expect preamble
          self.mdv_state_preamble=1
          self.mdv_state_header=0
        else:
          print("data at 0x%X" % filedata.tell())
          filedata.readinto(mv[16:514])
          print(self.data_buf[0:514])
          self.mdv_state_sync=1
          self.mdv_state_header=1

  def run(self,filedata):
    print("running")
    for i in range(1280):
      self.get(filedata)

c=check_mdv()
fd=open("QUILL.MDV","rb")
#fd.seek(20000)
c.run(fd)
