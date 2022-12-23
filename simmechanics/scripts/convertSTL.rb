#!/usr/bin/env ruby

# Copyright 2021 Chris Polis
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
# WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

# convertSTL.rb - Converts STL files between binary and ASCII encoding
# by Chris Polis
#
# This script detects which encoding the stl file is in and converts it to
#  the opposite encoding and saves the file as *-ascii.stl or *-binary.stl.
#  I wrote this script to save disk space and bandwidth when using stl files.
#
# USAGE:
#  $ ruby convertSTL.rb [filename(s) of .stl to be converted]
#  or 'chmod +x' and run as ./convertSTL.rb
#
#  note: path wildcards, (e.g.: "*.stl") are supported and all matching
#        files will be converted


# Helper methods
class Float
    def to_sn # to scientific notation
      "%E" % self
    end

    def self.from_sn str # generate a float from scientific notation
      ("%f" % str).to_f
    end
  end

  # Pass in filename as only argument
  if ARGV.size == 0
    puts "Usage: ./converSTL.rb [stl filename]"
    exit
  end

  begin
    # Read all files matching arg, "foo.stl", "*.stl", "../stl/*", etc...
    ARGV.each do |originalFilename|
      original = File.new(originalFilename, "r")

      # Read first line - check binary or ASCII
      tempLine = original.gets
      if tempLine.start_with? "solid"
        outFilename = originalFilename.sub(/\.stl/i, '-binary.stl')
        puts "#{originalFilename} is in ASCII format, converting to BINARY"
        outFile = File.new(outFilename, "w")
        outFile.write("\0" * 80) # 80 bit header - ignored
        outFile.write("FFFF")   # 4 bit integer # of triangles - filled later
        triCount = 0

        # ASCII STL format (from Wikipedia):
        # solid name(optional)
        #
        # [foreach triangle]
        # facet normal ni nj nk
        # outer loop
        # vertex v1x v1y v1z
        # vertex v2x v2y v2z
        # vertex v3x v3y v3z
        # endloop
        # endfacet
        # endsolid name(optional)

        while temp = original.gets
          next if temp =~ /^\s*$/ or temp.include? 'endsolid' # ignore whitespace
          temp.sub! /facet normal/, ''
          normal = temp.split(' ').map{ |num| Float.from_sn num }
          triCount += 1
          temp = original.gets # 'outer loop'

          temp = original.gets
          vertexA = temp.sub(/vertex/, '').split(' ').map{ |num| Float.from_sn num }
          temp = original.gets
          vertexB = temp.sub(/vertex/, '').split(' ').map{ |num| Float.from_sn num }
          temp = original.gets
          vertexC = temp.sub(/vertex/, '').split(' ').map{ |num| Float.from_sn num }

          temp = original.gets # 'endsolid'
          temp = original.gets # 'endfacet'

          outFile.write(normal.pack("FFF"))
          outFile.write(vertexA.pack("FFF"))
          outFile.write(vertexB.pack("FFF"))
          outFile.write(vertexC.pack("FFF"))
          outFile.write("\0\0")
        end
        outFile.seek(80, IO::SEEK_SET)
        outFile.write([ triCount ].pack("V"))
        outFile.close
        File.rename(outFilename, originalFilename)
      else
        puts "#{originalFilename} is in BINARY format, skipping"
      end
      original.close
    end
  rescue => error
    puts "Error: #{error}"
  end
