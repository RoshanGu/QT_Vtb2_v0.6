#!/usr/bin/python

import sys
import re

x2v = { '0': '0',
        '1': '73',
        }

def main(argv):
        if len(argv) != 2:
                print "Usage ./ucf2ioplace.py <UCF file> <PKG file> > output.ioplace"
                return

        ucf = argv[0]
        template = argv[1]

        loc2vpr = {}
        r = re.compile(r"pin IOB_X(\d+)Y(\d+) (.*)")
        with open(template) as fp:
                for line in fp:
                        line = line.strip()
                        m = r.match(line)
                        if m:
                            (x,y,loc) = m.groups()
                            y = int(y)
                            ymod2 = y%2
                            loc2vpr[loc] = [x2v[x],str(y+ymod2),str(1-ymod2)]

        print "# VPR pin constraints file generated from '%s' using '%s' as template" % (ucf,template)
        print ""
        print "##block name    x       y       subblk  block number"
        print "#----------     --      --      ------  ------------"
        print ""

        notfound = 0
        r = re.compile(r"^NET\s+\"([^\"]+)\".*LOC\s*=\s*\"([^\"]+)\".*;")
        r2 = re.compile(r"(.*)[<\[](\d+)[\]>]")
        with open(ucf) as fp:
                for line in fp:
                        m = r.match(line)
                        if m:
                                net,loc = m.group(1,2)
                                # If this is a vector index, e.g. foo[3]
                                # then transform to foo~3
                                m2 = r2.match(net)
                                if loc not in loc2vpr:
                                        print >>sys.stderr, "WARNING: LOC=\"%s\" not found, ignoring!" % loc
                                        notfound += 1
                                        continue

                                print '\t'.join( [net] + loc2vpr[loc] )
                                print 'out:'+'\t'.join( [net] + loc2vpr[loc] )
                                if m2:
                                    net = m2.group(1) + "~" + m2.group(2)
                                print 'top^'+'\t'.join( [net] + loc2vpr[loc] )
                                print 'out:top^'+'\t'.join( [net] + loc2vpr[loc] )
        if notfound:
                print >>sys.stderr, "WARNING: %d LOCs not found in template!" % notfound
                        

if __name__ == "__main__":
        main(sys.argv[1:])
