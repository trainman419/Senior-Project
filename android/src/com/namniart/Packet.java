package com.namniart;

import java.util.LinkedList;
import java.util.List;

public class Packet {
	private List<Byte> data;
	private static final byte ESC = 0x1b;
	
	// constructors
	public Packet() {
		data = new LinkedList<Byte>();
	}
	
	public Packet(byte[] in) {
		data = new LinkedList<Byte>();
		for( byte b : in) {
			data.add(b);
		}
	}
	
	public Packet(List<Byte> in) {
		data = new LinkedList<Byte>(in); // make our own copy because we will modify it
	}
	
	// utility methods
	public byte[] toByteArray() {
		byte[] out = new byte[data.size()];
		int i=0;
		for(byte b : data) {
			out[i] = b;
			i++;
		}
		return out;
	}
	
	// append methods
	public void append(byte b) {
		if( b != '\r' && b != ESC ) {
			data.add(b);
		} else {
			data.add(ESC);
			data.add((byte)(b ^ ESC));
		}
	}
	
	public void append(int a) {
		for(int i=0; i<4; i++) {
			append((byte)(a & 0xFF));
			a >>= 8;
		}
	}
	
	public void append(float f) {
		append(Float.floatToIntBits(f));
	}
	
	public void finish() {
		data.add((byte)'\r');
	}
	
	// read methods
	public byte reads8() {
		byte b = data.remove(0);
		if( b == ESC ) {
			b ^= data.remove(0);
		}
		return b;
	}
	
	public int reads32() {
		int res = 0;
		StringBuffer s = new StringBuffer("Got data: ");
		for( int i=0; i<4; i++ ) {
			byte b = reads8();
			s.append((int)b).append(" ");
			res |= (((int)b) & 0xFF) << (8*i);
		}
		System.out.println(s);
		return res;
	}
	
	public int sz() {
		return data.size();
	}
}
