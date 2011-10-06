#!/usr/bin/env python

#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

__author__ = "mferguson@willowgarage.com (Michael Ferguson)"

__usage__ = """
make_library.py generates the Arduino rosserial library files.  It 
requires the location of your arduino libraries folder and the name of 
one or more packages for which you want to make libraries.  

rosrun rosserial_client make_library.py <library_path>  pkg_name [pkg2 pkg3 ...]
"""

import roslib; roslib.load_manifest("rosserial_client")
import rospy

import os, sys, subprocess, re

ros_types = {
    'bool'  :   ('bool',           1),
    'byte'  :   ('unsigned char',  1),
    'char'  :   ('char',           1),
    'int8'  :   ('int8_t',         1),
    'uint8' :   ('uint8_t',        1),
    'int16' :   ('int16_t',        2),
    'uint16':   ('uint16_t',       2),
    'int32':    ('int32_t',        4),
    'uint32':   ('uint32_t',       4),
    'float32':  ('float',          4)
}

def type_to_var(ty):
    lookup = {
        1 : 'uint8_t',
        2 : 'uint16_t',
        4 : 'uint32_t'
    }
    return lookup[ty]


#####################################################################
# Data Types

class EnumerationType:
    """ For data values. """
    
    def __init__(self, name, ty, value):
        self.name = name
        self.type = ty
        self.value = value
    
    def make_declaration(self, f):
        f.write('      enum { %s = %s };\n' % (self.name, str(self.value)))    

class PrimitiveDataType:
    """ Our datatype is a C/C++ primitive. """    

    def __init__(self, name, ty, bytes, offset):
        self.name = name
        self.type = ty
        self.bytes = bytes
        self.offset = offset

    def make_declaration(self, f):
        #f.write('      %s %s;\n' % (self.type, self.name) )
        f.write('      class {\n')
        f.write('        public:\n')
        f.write('          %s operator=(const %s & other) {\n' % (self.type, self.type))
        f.write('              for(size_t i=0; i<sizeof(%s); ++i) {\n' % self.type)
        f.write('                  buffer[i+%d] = (other >> (i*8)) & 0xFF;\n' % self.offset)
        f.write('              }\n')
        f.write('              return other;\n')
        f.write('          }\n')
        f.write('          operator %s() {\n' % self.type)
        f.write('              %s ret = 0;\n' % self.type)
        f.write('              for(size_t i=0; i<sizeof(%s); ++i) {\n' % self.type)
        f.write('                  ret |= buffer[i+%d] << (i*8); \n' % self.offset)
        f.write('              }\n')
        f.write('              return ret;\n')
        f.write('          }\n')
        f.write('      } %s;\n' % self.name)

    def make_constructor(self):
        # do nothing for basic types
        return ""

class MessageDataType(PrimitiveDataType):
    """ For when our data type is another message. """
    def make_declaration(self, f):
       f.write('      %s %s;\n' % (self.type, self.name))

    def make_constructor(self):
       return "%s(buffer + %d)" % (self.name, self.offset)

# TODO: update this
class StringDataType(PrimitiveDataType):
    """ Need to convert to signed char *. """

    def make_declaration(self, f):
        f.write('      char * %s;\n' % self.name)

    def serialize(self, f):
        cn = self.name.replace("[","").replace("]","")
        f.write('      long * length_%s = (long *)(outbuffer + offset);\n' % cn)
        f.write('      *length_%s = strlen( (const char*) this->%s);\n' % (cn,self.name))
        f.write('      offset += 4;\n')
        f.write('      memcpy(outbuffer + offset, this->%s, *length_%s);\n' % (self.name,cn))
        f.write('      offset += *length_%s;\n' % cn)

    def deserialize(self, f):
        cn = self.name.replace("[","").replace("]","")
        f.write('      uint32_t length_%s = *(uint32_t *)(inbuffer + offset);\n' % cn)
        f.write('      offset += 4;\n')
        f.write('      for(unsigned int k= offset; k< offset+length_%s; ++k){\n'%cn) #shift for null character
        f.write('          inbuffer[k-1]=inbuffer[k];\n')
        f.write('           }\n')
        f.write('      inbuffer[offset+length_%s-1]=0;\n'%cn)
        f.write('      this->%s = (char *)(inbuffer + offset-1);\n' % self.name)
        f.write('      offset += length_%s;\n' % cn)

# TODO: update this
class TimeDataType(PrimitiveDataType):

    def __init__(self, name, ty, bytes, offset):
        self.name = name
        self.type = ty
        self.sec = PrimitiveDataType(name+'.sec','unsigned long',4, offset)
        self.nsec = PrimitiveDataType(name+'.nsec','unsigned long',4, offset+4)
        self.offset = offset

    def make_declaration(self, f):
        f.write('      %s %s;\n' % (self.type, self.name))

    def serialize(self, f):
        self.sec.serialize(f)
        self.nsec.serialize(f)

    def deserialize(self, f):
        self.sec.deserialize(f)
        self.nsec.deserialize(f)

# TODO: update this
class ArrayDataType(PrimitiveDataType):

    def __init__(self, name, ty, bytes, cls, array_size=None):
        self.name = name
        self.type = ty  
        self.bytes = bytes
        self.size = array_size
        self.cls = cls 

    def make_declaration(self, f):
        c = self.cls("*"+self.name, self.type, self.bytes, 0) # FIXME FIXME
        if self.size == None:
            f.write('      unsigned char %s_length;\n' % self.name)
            f.write('      %s st_%s;\n' % (self.type, self.name)) # static instance for copy
            f.write('      %s * %s;\n' % (self.type, self.name))
        else:
            f.write('      %s %s[%d];\n' % (self.type, self.name, self.size))
    
    def serialize(self, f):
        c = self.cls(self.name+"[i]", self.type, self.bytes)
        if self.size == None:
            # serialize length
            f.write('      *(outbuffer + offset++) = %s_length;\n' % self.name)
            f.write('      *(outbuffer + offset++) = 0;\n')
            f.write('      *(outbuffer + offset++) = 0;\n')
            f.write('      *(outbuffer + offset++) = 0;\n')
            f.write('      for( unsigned char i = 0; i < %s_length; i++){\n' % self.name)
            c.serialize(f)
            f.write('      }\n')
        else:
            f.write('      unsigned char * %s_val = (unsigned char *) this->%s;\n' % (self.name, self.name))    
            f.write('      for( unsigned char i = 0; i < %d; i++){\n' % (self.size) )
            c.serialize(f)            
            f.write('      }\n')
        
    def deserialize(self, f):
        if self.size == None:
            c = self.cls("st_"+self.name, self.type, self.bytes)
            # deserialize length
            f.write('      unsigned char %s_lengthT = *(inbuffer + offset++);\n' % self.name)
            f.write('      if(%s_lengthT > %s_length)\n' % (self.name, self.name))
            f.write('        this->%s = (%s*)realloc(this->%s, %s_lengthT * sizeof(%s));\n' % (self.name, self.type, self.name, self.name, self.type))
            f.write('      offset += 3;\n')
            f.write('      %s_length = %s_lengthT;\n' % (self.name, self.name))
            # copy to array
            f.write('      for( unsigned char i = 0; i < %s_length; i++){\n' % (self.name) )
            c.deserialize(f)
            f.write('        memcpy( &(this->%s[i]), &(this->st_%s), sizeof(%s));\n' % (self.name, self.name, self.type))                     
            f.write('      }\n')
        else:
            c = self.cls(self.name+"[i]", self.type, self.bytes)
            f.write('      unsigned char * %s_val = (unsigned char *) this->%s;\n' % (self.name, self.name))    
            f.write('      for( unsigned char i = 0; i < %d; i++){\n' % (self.size) )
            c.deserialize(f)            
            f.write('      }\n')


#####################################################################
# Messages

class Message:    
    """ Parses message definitions into something we can export. """

    def __init__(self, name, package, definition):

        self.name = name            # name of message/class
        self.package = package      # package we reside in
        self.includes = list()      # other files we must include

        self.data = list()          # data types for code generation
        self.enums = list()

        offset = 0

        # parse definition
        for line in definition:
            # prep work
            line = line.strip().rstrip()    
            value = None
            if line.find("#") > -1:
                line = line[0:line.find("#")]
            if line.find("=") > -1:
                value = int(line[line.find("=")+1:])
                line = line[0:line.find("=")]
            
            # find package/class name   
            line = line.replace("\t", " ")
            l = line.split(" ")
            while "" in l:
                l.remove("")
            if len(l) < 2:
                continue
            ty, name = l[0:2]
            if value != None:
                self.enums.append( EnumerationType(name, ty, value))            
                continue

            try:
                type_package, type_name = ty.split("/")
            except:
                type_package = None
                type_name = ty
            type_array = False
            if type_name.find('[') > 0:
                type_array = True   
                try:
                    type_array_size = int(type_name[type_name.find('[')+1:type_name.find(']')])
                except:
                    type_array_size = None
                type_name = type_name[0:type_name.find('[')]

            # convert to C/Arduino type if primitive, expand name otherwise
            try:
                # primitive type
                cls = PrimitiveDataType
                code_type = type_name
                size = 0
                if type_package:
                    cls = MessageDataType                    
                elif type_name == 'time':
                    cls = TimeDataType
                    code_type = 'ros::Time'
                    if "ros/time" not in self.includes:
                        self.includes.append("ros/time")
                elif type_name == 'duration':
                    cls = TimeDataType
                    code_type = 'ros::Duration'
                    if "ros/duration" not in self.includes:
                        self.includes.append("ros/duration")
                elif type_name == 'string':
                    cls = StringDataType
                    code_type = 'char*'
                else:
                    code_type = ros_types[type_name][0]
                    size = ros_types[type_name][1]
                if type_array:
                    self.data.append( ArrayDataType(name, code_type, size, cls, type_array_size ) )
                else:
                    self.data.append( cls(name, code_type, size, offset) )
                offset += size
            except:
                if type_name == 'Header':
                    self.data.append( MessageDataType(name, 'std_msgs::Header', 0, offset) )
                    if "std_msgs/Header" not in self.includes:
                        self.includes.append("std_msgs/Header")
                else:
                    if type_package == None or type_package == package:
                        type_package = package  
                        cls = MessageDataType
                        if self.package+"/"+type_name not in self.includes:
                            self.includes.append(self.package+"/"+type_name)
                    if type_package+"/"+type_name not in self.includes:
                        self.includes.append(type_package+"/"+type_name)
                    if type_array:
                        self.data.append( ArrayDataType(name, type_package + "::" + type_name, size, cls, type_array_size) )
                    else:
                        self.data.append( MessageDataType(name, type_package + "::" + type_name, 0, offset) )
        #print ""


    def _write_std_includes(self, f):
        f.write('#include <stdint.h>\n')
        f.write('#include <string.h>\n')
        f.write('#include <stdlib.h>\n')
        f.write('#include "../ros/msg.h"\n')

    def _write_msg_includes(self,f):
        for i in self.includes:
            f.write('#include "%s.h"\n' % i)

    def _write_constructor(self, f):
        inits = []
        for d in self.data:
            init = d.make_constructor()
            if len(init) > 0:
                inits.append(init)
        f.write('      %s(char * b) : buffer(b)' % self.name) 
        if len(inits) > 0:
            for init in inits:
               f.write(', %s' % init)
        f.write(' {}\n')
            
    def _write_data(self, f):
        for d in self.data:
            d.make_declaration(f)
        for e in self.enums:
            e.make_declaration(f)
            
    def _write_getType(self, f):
        f.write('      const char * getType(){ return "%s/%s"; };\n'%(self.package, self.name))

    def _write_impl(self, f):
        f.write('  class %s : public ros::Msg\n' % self.name)
        f.write('  {\n')
        f.write('    protected:\n')
        f.write('      char * buffer;\n');
        f.write('    public:\n')
        self._write_constructor(f)
        self._write_data(f)
        self._write_getType(f)
        f.write('\n')
        f.write('  };\n')
        
    def make_header(self, f):
        f.write('#ifndef ros_%s_%s_h\n'%(self.package, self.name))
        f.write('#define ros_%s_%s_h\n'%(self.package, self.name))
        f.write('\n')
        self._write_std_includes(f)
        self._write_msg_includes(f)
       
        f.write('\n')
        f.write('namespace %s\n' % self.package)
        f.write('{\n')
        f.write('\n')
        self._write_impl(f)
        f.write('\n')
        f.write('}\n')

        f.write('#endif')

class Service:
    def __init__(self, name, package, definition):
        """ 
        @param name -  name of service
        @param package - name of service package
        @param definition - list of lines of  definition
        """
        
        self.name = name
        self.package = package
        
        sep_line = None
        sep = re.compile('---*')
        for i in range(0, len(definition)):
            if (None!= re.match(sep, definition[i]) ):
                sep_line = i
                break
        self.req_def = definition[0:sep_line]
        self.resp_def = definition[sep_line+1:]
        
        self.req = Message(name+"Request", package, self.req_def)
        self.resp = Message(name+"Response", package, self.resp_def)
        
    def make_header(self, f):
        f.write('#ifndef ros_SERVICE_%s_h\n' % self.name)
        f.write('#define ros_SERVICE_%s_h\n' % self.name)
        
        self.req._write_std_includes(f)
        includes = self.req.includes
        includes.extend(self.resp.includes)
        includes = list(set(includes))
        for inc in includes:
            f.write('#include "%s.h"\n' % inc)
            
        f.write('\n')
        f.write('namespace %s\n' % self.package)
        f.write('{\n')
        f.write('\n')       
        f.write('static const char %s[] = "%s/%s";\n'%(self.name.upper(), self.package, self.name))
        
        def write_type(out, name):
            out.write('    const char * getType(){ return %s; };\n'%(name))
        _write_getType = lambda out: write_type(out, self.name.upper())
        self.req._write_getType = _write_getType
        self.resp._write_getType = _write_getType
        
        f.write('\n')
        self.req._write_impl(f)
        f.write('\n')
        self.resp._write_impl(f)
        f.write('\n')
        f.write('}\n')

        f.write('#endif')
        
   
        
#####################################################################
# Core Library Maker

class ArduinoLibraryMaker:
    """ Create an Arduino Library from a set of Message Definitions. """

    def __init__(self, package):
        """ Initialize by finding location and all messages in this package. """
        self.name = package
        print "\nExporting " + package +"\n", 

        self.pkg_dir = roslib.packages.get_pkg_dir(package)
        
        sys.stdout.write('Messages:\n    ')
        # find the messages in this package
        self.messages = list()
        if (os.path.exists(self.pkg_dir+"/msg")):
			for f in os.listdir(self.pkg_dir+"/msg"):
				if f.endswith(".msg"):
					# add to list of messages
					print "%s," % f[0:-4],
					definition = open(self.pkg_dir + "/msg/" + f).readlines()
					self.messages.append( Message(f[0:-4], self.name, definition) )
			print "\n"
     
        sys.stdout.write('Services:\n    ')
        # find the services in this package
        self.services = list()
        if (os.path.exists(self.pkg_dir+"/srv/")):
			for f in os.listdir(self.pkg_dir+"/srv"):
				if f.endswith(".srv"):
					# add to list of messages
					print "%s," % f[0:-4],
					definition = open(self.pkg_dir + "/srv/" + f).readlines()
					self.messages.append( Service(f[0:-4], self.name, definition) )
			print "\n"

    def generate(self, path_to_output):
        """ Generate header and source files for this package. """

        # generate for each message
        for msg in self.messages:
            if not os.path.exists(path_to_output + "/" + self.name):
                os.makedirs(path_to_output + "/" + self.name)
            header = open(path_to_output + "/" + self.name + "/" + msg.name + ".h", "w")
            msg.make_header(header)
            header.close()

    
if __name__=="__main__":

    # get path to arduino sketchbook
    
    if (len(sys.argv) <3):
        print __usage__
        exit()
    
    path = sys.argv[1]
    if path[-1] == "/":
        path = path[0:-1]
    path += "/ros_lib2"
    print "\nExporting to %s" % path

    # make libraries
    packages = sys.argv[2:]
    for msg_package in packages:
        lm = ArduinoLibraryMaker(msg_package)
        lm.generate(path)

