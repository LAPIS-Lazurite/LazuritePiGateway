#! /usr/bin/ruby
# -*- coding: utf-8; mode: ruby -*-
# Function:
#   Lazurite Sub-GHz/Lazurite Pi Gateway Sample program
#   SerialMonitor.rb

require 'net/http'
require 'date'
##
# BP3596 API
##
class BP3596PipeApi
  ##
  # func   : Read the data from the receiving pipe
  # input  : Receive pipe fp
  # return : Receive data
  ##
  def read_device(fp)
    # Data reception wait (timeout = 100ms)
    ret = select([fp], nil, nil, 0.1)

    # Reads the size of the received data
    len = fp.read(2)
    if ((len == "") || (len == nil)) then # read result is empty
      return -1
    end
    size =  len.unpack("S*")[0]

    # The received data is read
    recv_buf = fp.read(size)
    if ((recv_buf == "") || (recv_buf == nil)) then # read result is empty
      return -1
	end

    return recv_buf
  end
  def BinaryMonitor(raw)
    len = raw.length
    header = raw.unpack("H4")[0]

	# unsupported format
	if header != "21a8" then
	  unsupported_format(raw)
	  return
	end

	# supported format
    seq = raw[2].unpack("H2")[0]

	# PANID
    myPanid = raw[3..4].unpack("S*")[0]

	# RX Address
	rxAddr = raw[5..6].unpack("S*")[0]

	# TX Address
	txAddr = raw[7..8].unpack("S*")[0]

	# 
	print(sprintf("PANID=0x%04X, rxAddr=0x%04X, txAddr=0x%04X, DATA:: ",myPanid, rxAddr, txAddr))

	for num in 9..len-2 do
	  print(raw[num].unpack("H*")[0]," ")
    end
	print("\n")
  end

  # printing unsupported format
  def unsupported_format(raw)
    data = raw.unpack("H*")
	print("unsupported format::",data,"\n")
  end
end

##
# Main function
##
class MainFunction
  ### Variable definition
  bp3596_dev  = "/dev/bp3596" # Receiving pipe
  finish_flag = 0             # Finish flag

  # Process at the time of SIGINT Receiving
  Signal.trap(:INT){
    finish_flag=1
  }

  # Open the Receiving pipe
  bp3596_fp = open(bp3596_dev, "rb")

  bp_api = BP3596PipeApi.new

  # Loop until it receives a SIGINT
  while finish_flag==0 do
    # Read device
    recv_buf = bp_api.read_device(bp3596_fp)
    if recv_buf == -1
      next
    end
    # Create a transmit buffer from the receive buffer
	bp_api.BinaryMonitor(recv_buf)
  end
  # Close the Receiving pipe
  bp3596_fp.close
end
