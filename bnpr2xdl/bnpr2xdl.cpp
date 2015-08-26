#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <iostream>
#include <iomanip>

#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/predicate.hpp>
using boost::algorithm::starts_with;
using boost::algorithm::ends_with;
using boost::algorithm::to_upper_copy;

#include "bnpr2xdl.hpp"

const string design::whitespace = " \a\b\f\n\r\t\v";

inline string escape(const string& s) {
	string r(s);
	string::size_type i = 0;
	while (1) {
		i = r.find_first_of(':', i);
		if (i == string::npos)
			break;
		r.insert(i, "\\");
		i += 2;
	}
	return r;
}

design::design(
		const string& device,
		const string& package,
		const string& speed,
		const string& version = "v3.2") 
	: mDevice(device), mPackage(package), mSpeed(speed), mVersion(version), mDDB(device,package)
{
}

void design::operator()(const string& circuit) {
	parseNet(stringify(circuit, ".net"));
	parsePlace(stringify(circuit, ".place"));
	parseRoute(stringify(circuit, ".route"));
	parseBlif(stringify(circuit, ".pre-vpr.blif"));
	include();
}

void design::include(void) {
	const string &fn = stringify(mDesignPtr->getDevice(), mDesignPtr->getPackage(), "_include.xdl");
	ifstream fs(fn);
	if(!fs.good())
		return;
	
	cout << "Including instances from " << fn << "... " << endl;

	XdlImporter importer;
	importer(fs, fn);
	auto inDesignPtr = importer.getDesignPtr();
	auto it = inDesignPtr->instancesBegin(), ie = inDesignPtr->instancesEnd();
	for (; it != ie; ++it) {
		mDesignPtr->addInstance(*it);
	}

	fs.close();
}

void design::parseNet(const string &net) {
	//mXml = new xmlpp::TextReader(net);
	mXml = xmlNewTextReaderFilename(net.c_str());

	advance(XML_READER_TYPE_ELEMENT, 0, "block");
	assert(get_attribute("instance") == "FPGA_packed_netlist[0]");
	const string& name = get_attribute("name");

	mDesignPtr = Factory::newDesignPtr(name, mDevice, mPackage, mSpeed, mVersion);

	// Ignore FPGA inputs
	skip(XML_READER_TYPE_ELEMENT, 1, "inputs");

	// Ignore FPGA outputs
	skip(XML_READER_TYPE_ELEMENT, 1, "outputs");

	// Ignore FPGA clocks
	skip(XML_READER_TYPE_ELEMENT, 1, "clocks");

	// Parse all blocks;
	advance();
	do {
		check(XML_READER_TYPE_ELEMENT, 1, "block");
		const string &mode = get_attribute("mode");
		const string &instance = get_attribute("instance");
		const string &type = instance.substr(0,instance.find_first_of('['));

		if (type == "SLICEL")
			parseSLICEL();
		else if (type == "IOB")
			parseIOB();
		else if (type == "DSP48E1")
			parseDSP();
		else if (type == "BUFG")
			parseBUFG();
		else if (type == "PS7")
			parsePS7();
		else if (type == "RAMB36E1") {
			if (mode == "RAMB18E1x2")
				parseRAMB18x2();
			else
				parseRAMB36();
		}
		else throw;

		advance();
	} while(get_depth() > 0);

	check(XML_READER_TYPE_END_ELEMENT, 0, "block");

	//delete mXml;
	xmlFreeTextReader(mXml);
}

void design::parsePlace(const string &place) {
	const Tiles& tiles = mDDB.getTiles();

	map<TileIndex, vector<string> > tile2sites;
	genClbTile2Sites(tile2sites);

	map<tuple<int,int,int>, string> iob2site;
	readIob2Site(iob2site);

	ifstream fs(place);
	if (!fs) {
		cout << "parsePlace() failed: '" << place << "' not found!" << endl;
		throw;	
	}

	// Ignore the first five lines
	fs.ignore(1024, '\n');
	fs.ignore(1024, '\n');
	fs.ignore(1024, '\n');
	fs.ignore(1024, '\n');
	fs.ignore(1024, '\n');

	string block;
	int x, y, subblk;
	while (fs >> block >> x >> y >> subblk) {
		y -= 1;
		Design::InstanceSharedPtrConstIterator instPtrIt = mDesignPtr->findInstance(block);
		assert(instPtrIt != mDesignPtr->instancesEnd());
		InstanceSharedPtr instPtr = *instPtrIt;
		const string& type = instPtr->getType();
		if (type == "SLICEL") {
			string tile;
			TileIndex ti;
			for (const auto p : { "CLBLM", "CLBLL", "CLBLM_L", "CLBLM_R", "CLBLL_L", "CLBLL_R" }) {
				tile = stringify(p, "_X", x, "Y", y);
				ti = tiles.findTileIndex(tile);
				if (ti != TileIndex(-1))
					break;
			}
			assert(ti != TileIndex(-1));
			instPtr->setTile(tile);
			instPtr->setSite(tile2sites.at(ti)[subblk]);
		}
		else if (type == "DSP48E1") {
			string tile;
			TileIndex ti;
			for (const auto p : { "DSP", "DSP_L", "DSP_R" }) {
				tile = stringify(p, "_X", x, "Y", y);
				ti = tiles.findTileIndex(tile);
				if (ti != TileIndex(-1))
					break;
			}
			assert(ti != TileIndex(-1));
			instPtr->setTile(tile);
			instPtr->setSite(tile2sites.at(ti)[subblk]);
		}
		else if (type == "RAMB18E1") {
			string tile;
			TileIndex ti;
			for (const auto p : { "BRAM", "BRAM_L", "BRAM_R" }) {
				tile = stringify(p, "_X", x, "Y", y);
				ti = tiles.findTileIndex(tile);
				if (ti != TileIndex(-1))
					break;
			}
			assert(ti != TileIndex(-1));
			// FIXME: Fix ptc2name!!
			const auto& site = tile2sites.at(ti)[1-subblk];
			assert(starts_with(site, "RAMB18_"));
			instPtr->setTile(tile);
			instPtr->setSite(site);
			
			InstanceSharedPtr inst2Ptr = mSecondaryRAMB18.at(instPtr);
			if (inst2Ptr) {
				const auto& site2 = tile2sites.at(ti)[subblk];
				assert(starts_with(site, "RAMB18_"));
				inst2Ptr->setTile(tile);
				inst2Ptr->setSite(site2);
			}
		}
		else if (type == "RAMB36E1") {
			string tile;
			TileIndex ti;
			for (const auto p : { "BRAM", "BRAM_L", "BRAM_R" }) {
				tile = stringify(p, "_X", x, "Y", y);
				ti = tiles.findTileIndex(tile);
				if (ti != TileIndex(-1))
					break;
			}
			assert(ti != TileIndex(-1));
			instPtr->setTile(tile);
			// RAMB36 is after 2xRAMB18s
			const auto& site = tile2sites.at(ti)[2];
			assert(starts_with(site, "RAMB36_"));
			instPtr->setSite(site);
		}
		else if (type == "IOB") {
			string tile;
			if (mDDB.getDeviceName() == "xc7z020") {
				switch (x) {
					case 0: tile = stringify("LIOB33_X", x, "Y", y); break;
					case 73: tile = stringify("RIOB33_X", x, "Y", y); x = 1; break;
					default: throw;
				}
			}
			else if (mDDB.getDeviceName() == "xc6vlx240t") {
				switch (x) {
					case 0: tile = stringify("LIOB_X", x, "Y", y); break;
					case 41: tile = stringify("LIOB_FT_X", x, "Y", y); x = 1; break;
					case 57: tile = stringify("RIOB_X", x, "Y", y); x = 2; break;
					default: throw;
				}
				// FIXME: Fix ptc2name!!
				if (subblk == 0)
					y += 1;
				else if (subblk == 1)
					y -= 1;
				else throw;
			}
			else throw;
			assert(tiles.findTileIndex(tile) != TileIndex(-1));
			instPtr->setTile(tile);
			instPtr->setSite(iob2site.at(make_tuple(x,y,subblk)));
			cout << instPtr->getName() << " -> " << instPtr->getSite() << endl;
		}
		else if (type == "PS7") {
			if (x == 18)
				x = 32;
			else throw;
			const string& tile = stringify("PSS2_X",x,"Y",y);
			assert(tiles.findTileIndex(tile) != TileIndex(-1));
			instPtr->setTile(tile);
			string site = stringify(type, "_X", 0, "Y", 0);
			instPtr->setSite(site);
		}
		else if (type == "BUFG") {
			int z;
			string tile;

			if (mDDB.getDeviceName() == "xc7z020") {
				assert(x == 33);
				if (y == 46) {
					z = subblk;
					tile = "CLK_BUFG_BOT_R_X87Y48";
				}
				else if (y == 50) {
					z = subblk + 16;
					tile = "CLK_BUFG_TOP_R_X87Y53";
				}
				else throw;
			}
			else if (mDDB.getDeviceName() == "xc6vlx240t") {
				assert(x == 52);
				if (y == 118) {
					z = subblk;
					tile = "CMT_BUFG_BOT_X52Y118";
				}
				else if (y == 120) {
					z = subblk + 16;
					tile = "CMT_BUFG_TOP_X52Y120";
				}
				else throw;

			}
			assert(tiles.findTileIndex(tile) != TileIndex(-1));
			instPtr->setTile(tile);
			string site = stringify("BUFGCTRL_X0Y", z);
			instPtr->setSite(site);
		}
		else throw;
		fs.ignore(1024, '\n');
		//cout << instPtr->getType() << " " << instPtr->getName() << " @ " << instPtr->getTile() << " " << instPtr->getSite() << endl;
	}
	fs.close();
}

void design::parseBlif(const string &blif) {
	ifstream fs(blif);
	if (!fs) {
		cout << "parseBlif() failed: '" << blif << "' not found!" << endl;
		throw;
	}
	fs.ignore(1024, '\n');

	string begin, line;
	string mask, tti, tto, ttoLast;
	vector<string> vLine;
	vector<string> in2ipin;
	in2ipin.reserve(6);
	vector<bool> ipinUsed;
	ipinUsed.reserve(6);

	while (fs >> begin) {
		if (begin[0] != '.')
			fs.ignore(1024, '\n');
		else if (begin == ".names") {
			getline(fs, line);
			while (line.back() == '\\') {
				string l;
				getline(fs, l);
				line.pop_back();
				line += l;
			}
			static const regex re("\\s+");
			const string& tline = trim(line);
			sregex_token_iterator split_first{tline.begin(), tline.end(), re, -1}, split_last;
			vLine = {split_first, split_last};
			// Only one token, therefore constant (e.g. gnd), ignore
			if (vLine.size() == 1) {
				if (vLine[0] == "gnd" || vLine[0] == "vcc" || vLine[0] == "unconn")
					continue;
			}
			const string out = vLine.back();
			vLine.pop_back();
			InstanceSharedPtr instPtr;
			string config;
			auto it = mNet2InstCfg.find(out);
			if (it == mNet2InstCfg.end()) {
				assert(vLine.size() == 1);
				fs >> tti >> tto;
				assert(tti == "1");
				assert(tto == "1");
				const string& in = vLine.front();
				cout << "parseBlif(): Cannot find lut " << out << " (input net: " << in << ") --- assume trivial buffer, ignore" << endl;
				continue;
			}
			std::tie(instPtr,config) = it->second;
			assert(config[1] == '6' || config[1] == '5');

			// Must be a constant
			if (vLine.empty()) {
				fs >> mask;
				assert(mask == "0" || mask == "1");
			}
			else {
				const char &lut = config[0];
				const char &maxPin = instPtr->hasConfig(stringify(lut,"5LUT")) ? '5' : '6';
				in2ipin.clear();
				for (const auto &in : vLine) {
					Circuit::NetSharedPtrConstIterator it = mDesignPtr->findNet(in);
					assert(it != mDesignPtr->netsEnd());
					NetSharedPtr inNet = *it;

					string ipin;
					for (auto it = inNet->sinksBegin(), ie = inNet->sinksEnd(); it != ie; ++it) {
						if ((*it)->getInstancePtr().lock() == instPtr) {
							const PinName &pinName = (*it)->getPinName();
							if (pinName[0] == lut && pinName[1] >= '1' && pinName[1] <= maxPin) {
								if (ipin.empty())
									ipin = stringify('A', pinName[1]);
								else
									ipin += stringify("*A", pinName[1]);
							}
						}
					}
					assert(!ipin.empty());
					if (ipin.size() > 2)
						ipin = stringify("(", ipin, ")");
					in2ipin.push_back(ipin);
				}
				assert(in2ipin.size() == vLine.size());

				mask.clear();
				ttoLast.clear();
				ipinUsed.clear();
				while(fs >> tti >> tto) {
					assert(tto == "0" || tto == "1");
					if (!ttoLast.empty())
						assert(ttoLast == tto);
					if (mask.empty()) {
						if (tto == "0")
							mask += "~";
					}
					else
						mask += "+";
					mask += "(";
					if (ipinUsed.empty())
						ipinUsed.resize(tti.size(), false);
					else
						assert(tti.size() == ipinUsed.size());
					for (size_t i = 0; i < tti.size(); ++i) {
						const char &c = tti[i];
						if (i > 0)
							mask += "*";
						switch (c) {
							case '0': mask += stringify("~",in2ipin[i]); break;
							case '1': mask += in2ipin[i]; break;
							case '-': mask += "1"; break;
							default: throw;
						}
						if (c != '-')
							ipinUsed[i] = true;
					}
					mask += ")";
					fs.ignore(1024, '\n');
					if (fs.peek() == '.')
						break;
					ttoLast = tto;
				}

				for (size_t i = 0; i < ipinUsed.size(); ++i) {
					if (!ipinUsed[i])
						mask += stringify("+", in2ipin[i],"*~",in2ipin[i]);
				}
			}

			assert(instPtr->hasConfig(config));
			string n, v;
			instPtr->getConfig(config, n, v);
			instPtr->setConfig(config, n, stringify(v, "(", mask, ")"));
		}
	}
	fs.close();
}

void design::parseRoute(const string &route) {
	vector<Tilewire> inode2tw;
	int inodeOffset;
	readInode2Tw(inode2tw, inodeOffset);

	ifstream fs(route);
	if (!fs) {
		cout << "parseRoute() failed: '" << route << "' not found!" << endl;
		return;
	}

	map<string, string> tile2site;
	genIntTile2Site(tile2site);

	string begin, net;
	string node, rr_type;
	int inet, inode, xlow, ylow;
	WireUsage &wu = mDDB.getWireUsage();
	while (fs >> begin) {
		if (begin != "Net") {
			fs.ignore(1024, '\n');
		}
		else {
			fs >> inet >> net;

			net = net.substr(1,string::npos);
			// If ends in colon, means it's a global net
			if (net.back() == ':') {
				// But double check anyway
				string global, net, connecting;
				fs >> global >> net >> connecting;
				assert(global == "global");
				assert(net == "net");
				assert(connecting == "connecting:");
				continue;
			}
			assert(net.back() == ')');
			net.pop_back();
			fs.ignore(1024, '\n');

			NetSharedPtr netPtr;
			map< tuple<TileIndex,string>, vector<InstancePinSharedPtr> > tileLut2InstPin; 

			Circuit::NetSharedPtrIterator netPtrIt = mDesignPtr->findNet(net);
			if (netPtrIt == mDesignPtr->netsEnd()) {
				static const regex re("GLOBAL_LOGIC([01])_X(\\d+)Y(\\d+)");
				smatch m;
				bool b = regex_match(net, m, re);
				assert(b);
				
				const string type = m[1];
				const int x = boost::lexical_cast<int>(m[2]);
				const int y = boost::lexical_cast<int>(m[3])-1;
				InstanceSharedPtr instPtr = findOrCreateTieoff(tile2site,x,y);

				netPtr = torc::physical::Factory::newNetPtr(net);
				if (type == "0") netPtr->setNetType(torc::physical::eNetTypeGround);
				else if (type == "1") netPtr->setNetType(torc::physical::eNetTypePower);
				else throw;
				mDesignPtr->addNet(netPtr);

				torc::physical::InstancePinSharedPtr outPin;
				if (type == "0") {
					instPtr->setConfig("_GND_SOURCE","","HARD0");
					outPin = torc::physical::Factory::newInstancePinPtr(instPtr, "HARD0");

				}
				else if (type == "1") {
					instPtr->setConfig("_VCC_SOURCE","","HARD1");
					outPin = torc::physical::Factory::newInstancePinPtr(instPtr, "HARD1");
				}
				else throw;
				netPtr->addSource(outPin);

				// Populate tileLut2InstPin with the actual global net
				if (type == "0" || type == "1") {
					if (type == "0") net = "gnd";
					else if (type == "1") net = "vcc";
					else throw;
					netPtrIt = mDesignPtr->findNet(net);
				}
				assert(netPtrIt != mDesignPtr->netsEnd());
				genTileLut2InstPin(*netPtrIt, tileLut2InstPin);
			}
			else {
				netPtr = *netPtrIt;
				genTileLut2InstPin(netPtr, tileLut2InstPin);
			}

			Tilewire twLast;
			while (fs >> node >> inode >> rr_type) {
				assert(node == "Node:");
				fs.ignore(2, '(');
				fs >> xlow;
				fs.ignore(1, ',');
				fs >> ylow;

				if (rr_type == "CHANX" || rr_type == "CHANY") {
					const Tilewire &tw = inode2tw[inode-inodeOffset];
					// If its the first tilewire of the net, then
					// it must be the OPIN
					if (twLast.isUndefined()) {
						const ExtendedWireInfo ewi(mDDB, tw);
						// If pin is A, but routing starts at AMUX, add an A -> AMUX routethrough pip
						if (ends_with(ewi.mWireName, "MUX")) {
							assert(netPtr->hasOneSource());
							InstancePinSharedPtr srcPin = *netPtr->sourcesBegin();
							const string& srcPinName = srcPin->getPinName();
							if (srcPinName == "A" || srcPinName == "B" || srcPinName == "C" || srcPinName == "D") {
								string srcPinWireName(ewi.mWireName);
								srcPinWireName.erase(srcPinWireName.length()-3, string::npos);
								const Pip& p = Factory::newPip(ewi.mTileName, srcPinWireName,
										ewi.mWireName, ePipUnidirectionalBuffered);
								assert(!netPtr->containsPip(p));
								netPtr->addPip(p);
							}
						}
					}
					// If tilewire has already been marked as used (by us)
					// then it means this is a branch point -- just ignore it
					// so that it becomes twLast for use on the next tilewire
					else if (!wu.isUsed(tw)) {
						const Arc& a = findArc(twLast, tw);
						assert(!a.isUndefined());
						wu.use(twLast);
						wu.use(tw);
						//ddb.useSegment(tw);

						ExtendedWireInfo ewiSrc(mDDB, a.getSourceTilewire());
						ExtendedWireInfo ewiSnk(mDDB, a.getSinkTilewire());
						const Pip& p = Factory::newPip(ewiSrc.mTileName, ewiSrc.mWireName,
								ewiSnk.mWireName, ePipUnidirectionalBuffered);
						assert(!netPtr->containsPip(p));
						netPtr->addPip(p);
					}
					twLast = tw;
				}
				// If we arrive at an IPIN, look at the last tilewire and do
				// any pin swapping necessary
				else if (rr_type == "IPIN") {
					assert(!twLast.isUndefined());
					const auto& ewi = ExtendedWireInfo(mDDB, twLast);
					const string wireName(ewi.mWireName);
					if (isClb(ewi.mTileTypeName) 
							&& wireName.back() >= '1' && wireName.back() <= '6') {
						// From "CLBLL_L_A6" extract the "L_A" which indicates the site (L/M/LL) and the LUT
						const auto& tileLut = make_tuple(ewi.mTileIndex, wireName.substr(6, wireName.length()-6-1));
						auto it = tileLut2InstPin.find(tileLut);
						if (it == tileLut2InstPin.end()) {
							cout << netPtr->getName() << " to " << get<1>(tileLut) << " of " << ewi << " not found!" << endl;
							throw;
						}
						assert(it != tileLut2InstPin.end());
						const auto& newPinName = wireName.substr(wireName.length()-2, string::npos);
						const char& lut = newPinName[0];
						auto jt = it->second.rbegin(), je = it->second.rend();
						for (; jt != je; ++jt) {
							InstancePinSharedPtr instPinPtr = *jt;
							// From "CLBLL_L_A6" extract the "A6"
							const auto oldPinName = instPinPtr->getPinName();
							InstanceSharedPtr instPtr = instPinPtr->getInstancePtr().lock();
							// Do not swap vcc nets that go from/to [ABCD]6 of fractured LUTs
							if (netPtr->getNetType() == torc::physical::eNetTypePower 
									&& (oldPinName[1] == '6'|| newPinName[1] == '6')) {
								const string& cfg6 = stringify(lut,"6LUT");
								string n, v;
								instPtr->getConfig(cfg6, n, v);
								if (v.find("(A6+~A6)") != string::npos)
									continue;
							}
							if (oldPinName != newPinName) {
								instPinPtr->setPinName(newPinName);

								// Check if LUT is a "pack-thru" (i.e. mode == "wire")
								// and pin swap that if necessary
								const string& cfg6 = stringify(lut,"6LUT");
								string n, v;
								instPtr->getConfig(cfg6, n, v);
								if (starts_with(n, escape(net))
										&& regex_match(v, regex(stringify("#LUT:O6=(\\(A6\\+~A6\\)\\*)?A",oldPinName[1]))))
									instPtr->setConfig(cfg6, n, stringify(v.substr(0,v.length()-1), newPinName[1]));
								const string& cfg5 = stringify(lut,"5LUT");
								instPtr->getConfig(cfg5, n, v);
								if (starts_with(n, escape(net))
										&& regex_match(v, regex(stringify("#LUT:O5=A",oldPinName[1]))))
									instPtr->setConfig(cfg5, n, stringify("#LUT:O5=A",newPinName[1]));

								it->second.erase(--jt.base());
								break;
							}
						}

					}
					twLast = Tilewire();
				}

				fs.ignore(1024, '\n');

				const int &p = fs.peek();
				if (p == EOF)
					break;
				if (p != 'N') {
					if (p != '\n')
						cout << p << " (" << static_cast<int>(p) << ")" << endl;
					assert(p == '\n');
					break;
				}
			}
		}
	}
}


Arc design::findArc(const Tilewire &twSrc, const Tilewire &twSnk) {
	TilewireVector vSrcSeg, vSnkSeg;
	mDDB.expandSegment(twSrc, vSrcSeg);
	mDDB.expandSegment(twSnk, vSnkSeg);
	for (const auto &tw : vSnkSeg) {
		TilewireVector v;
		mDDB.expandTilewireSources(tw, v);
		auto it = find_first_of(v.begin(), v.end(), vSrcSeg.begin(), vSrcSeg.end());
		if (it != v.end()) {
			return Arc(*it,tw);
		}
	}
	throw;
}


void design::readInode2Tw(vector<Tilewire> &inode2tw, int &inodeOffset) {
	const string& fn = stringify(mDesignPtr->getDevice(), mDesignPtr->getPackage(), ".tws");
	ifstream fs(fn, ios::binary);
	if (!fs) {
		cout << "readInode2Tw() failed: '" << fn << "' not found!" << endl;
		throw;
	}
	fs.seekg(0, fs.end);
	const size_t size = (fs.tellg()/sizeof(Tilewire))-sizeof(int);
	inode2tw.resize(size);
	fs.seekg(0, fs.beg);
	fs.read(reinterpret_cast<char*>(&inodeOffset), sizeof(int));
	fs.read(reinterpret_cast<char*>(inode2tw.data()), inode2tw.size()*sizeof(Tilewire));
	fs.close();
}

void design::addInputPort(InstanceSharedPtr instPtr) {
	const int &depth = get_depth();
	check(XML_READER_TYPE_ELEMENT, depth, "port");
	string name = get_attribute("name");
	advance(XML_READER_TYPE_TEXT, depth+1, "#text");
	const string &text = trim(get_value());
	skip(depth+1);

	static const regex re("\\s+");
	sregex_token_iterator split_first{text.begin(), text.end(), re, -1}, split_last;
	const vector<string> &vText = {split_first, split_last};

	if (vText.size() == 1) {
		if (vText[0] != "open")
			addSink(vText[0], instPtr, name);
		return;
	}
		
	// If there a space exists, then this must be a vector,
	// so start counting from -1
	int index = 0;
	// By default, increment by 1
	int incr = 1;

	// Exception is [ABCD]6-1, which starts at 6 and decrements
	if (name == "A6-1" || name == "B6-1" || name == "C6-1" || name == "D6-1") {
		name = name[0];
		index = 6;
		incr = -1;
	}

	for (const auto &net : vText) {
		if (net != "open")
			addSink(net, instPtr, stringify(name, index));
		index += incr;
	}
}

void design::parseInputPort(map<string,string>& inputs) {
	const int& depth = get_depth();
	check(XML_READER_TYPE_ELEMENT, depth, "port");
	string name = get_attribute("name");
	// Remove "s0_" and "s1_" used to diffrentiate
	// RAMB18 ports from RAMB36 ones
	if (starts_with(name, "s0_") || starts_with(name, "s1_"))
		name = name.substr(3,string::npos);
	advance(XML_READER_TYPE_TEXT, depth+1, "#text");
	const string& text = get_value();

	static const regex re("\\s+");
	sregex_token_iterator split_first{text.begin(), text.end(), re, -1}, split_last;
	const vector<string> &vText = {split_first, split_last};

	if (vText.size() == 1) {
		if (vText[0] != "open") {
			bool b = inputs.insert(make_pair(name,vText[0])).second;
			assert(b);
		}
		return;
	}

	// If there a space exists, then this must be a vector,
	// so start counting from -1
	int index = 0;
	// By default, increment by 1
	int incr = 1;

	// Exception is [ABCD]6-1, which starts at 6 and decrements
	if (name == "A6-1" || name == "B6-1" || name == "C6-1" || name == "D6-1") {
		name = name[0];
		index = 6;
		incr = -1;
	}
		
	for (const auto &net : vText) {
		if (net != "open") {
			bool b = inputs.insert(make_pair(stringify(name, index),net)).second;
			assert(b);
		}
		index += incr;
	}
}

void design::addOutputPort(InstanceSharedPtr instPtr) {
	const int &depth = get_depth();
	check(XML_READER_TYPE_ELEMENT, depth, "port");
	const string &name = get_attribute("name");
	advance(XML_READER_TYPE_TEXT, depth+1, "#text");
	const string &text = get_value();
	skip(depth+1);

	static const regex re("\\s+");
	sregex_token_iterator split_first{text.begin(), text.end(), re, -1}, split_last;
	const vector<string> &vText = {split_first, split_last};

	if (vText.size() == 1) {
		if (vText[0] != "open")
			addSource(vText[0], instPtr, to_upper_copy(name));
		return;
	}
		
	// If there a space exists, then this must be a vector,
	// so start counting from 0
	int index = 0;
	// By default, increment by 1
	int incr = 1;

	for (const auto &net : vText) {	
		if (net != "open") {
			addSource(net, instPtr, to_upper_copy(stringify(name, index)));
		}
		index += incr;
	}
}

void design::parseOutputPort(map<string,string>& outputs) {
	const int& depth = get_depth();
	check(XML_READER_TYPE_ELEMENT, depth, "port");
	const string& name = get_attribute("name");
	advance(XML_READER_TYPE_TEXT, depth+1, "#text");
	const string& text = get_value();
	// Check only one signal
	assert(text.find_first_of(whitespace) == string::npos);

	bool b = outputs.insert(make_pair(name,text)).second;
	assert(b);

	skip(depth);
}

void design::parseLUT6(
		InstanceSharedPtr instPtr, 
		const char& lut, 
		const string& bleName,
		const map<string,string>& parentInputs,
		const map<string,string>& parentOutputs
		) {
	string name = get_attribute("name");
	const string &instance = get_attribute("instance");
	const string &mode = get_attribute("mode");
	// Blank LUT
	if (name == "open") {
		if (mode != "wire") {
			assert(mode.empty());
			return;
		}
	}
	string net = name;

	// Parse LUT inputs
	advance(XML_READER_TYPE_ELEMENT, 4, "inputs");
	advance(XML_READER_TYPE_ELEMENT, 5, "port");
	assert(get_attribute("name") == "in");
	advance(XML_READER_TYPE_TEXT, 6, "#text");
	char ipin = '\0';
	if (mode == "wire") {
		const string &inputs = get_value();
		static const regex re("\\s+");
		sregex_token_iterator split_first{inputs.begin(), inputs.end(), re, -1}, split_last;
		const vector<string> &vInputs = {split_first, split_last};
		if (instance == "LUT6[0]") {
			assert(vInputs.size() == 6);
			ipin = '6';
		}
		else if (instance == "LUT5[0]" || instance == "LUT5[1]") {
			assert(vInputs.size() == 5);
			ipin = '5';
		}
		else throw;
		
		bool found = false;
		for (const auto &v : vInputs) {
			assert(ipin > '0');
			if (v != "open") {
				assert(!found);
				found = true;
			}
			if (!found)
				--ipin;
		}
		assert(found);

		auto it = parentInputs.find(stringify(lut, ipin));
		assert(it != parentInputs.end());
		name = stringify(it->second, "|", instPtr->getName());
		net = it->second;
	}
	skip(4);

	assert(net != "open");

	// Ignore LUT outputs (because there is no ambiguity)
	skip(XML_READER_TYPE_ELEMENT, 4, "outputs");

	// Ignore LUT clocks (because there are none)
	skip(XML_READER_TYPE_ELEMENT, 4, "clocks");

	string outputs;
	if (starts_with(mode, "LUT")) {
		advance(XML_READER_TYPE_ELEMENT, 4, "block");
		assert(get_attribute("name") == name);
		assert(get_attribute("instance") == "lut[0]");

		// Ignore LUT inputs (because there is no ambiguity)
		skip(XML_READER_TYPE_ELEMENT, 5, "inputs");

		// Parse LUT outputs
		advance(XML_READER_TYPE_ELEMENT, 5, "outputs");
		advance(XML_READER_TYPE_ELEMENT, 6, "port");
		assert(get_attribute("name") == "out");
		advance(XML_READER_TYPE_TEXT, 7, "#text");
		outputs = get_value();
		// Check only one output exists, and same as LUT name
		assert(outputs.find_first_of(whitespace) == string::npos);
		assert(outputs == name);
	}

	if (instance == "LUT6[0]") {
		const string& config = stringify(lut, "6LUT");
		
		if (mode == "wire") {
			name = escape(stringify(name, "|rt", lut, "6"));
			assert(ipin);
			instPtr->setConfig(config, name, stringify("#LUT:O6=A",ipin));
		}
		else if (mode == "LUT6") {
			name = escape(name);
			instPtr->setConfig(config, name, "#LUT:O6=");
			bool b = mNet2InstCfg.insert(make_pair(net, make_tuple(instPtr,config))).second;
			assert(b);

			if (name == "gnd")
				instPtr->setConfig(config, name, "#LUT:O6=0");
			else if (name == "vcc")
				instPtr->setConfig(config, name, "#LUT:O6=1");
		}
		else throw;

		if (net != "gnd" && net != "vcc") {
			// Activate A if either A or AMUX are O6
			const string& out = parentOutputs.at(stringify("A"));
			const string& outMUX = parentOutputs.at(stringify("AMUX"));
			if (out == "LUT6[0].out[0]->A" || outMUX == "LUT6[0].out[0]->AMUX") {
				addSource(net, instPtr, stringify(lut));
				instPtr->setConfig(stringify(lut, "USED"), "", "0");
			}
		}

		if (!mBels.insert(name).second) {
			cout << config << ":" << name << " duplicated!" << endl;
			throw;
		}
	}
	else if (instance == "LUT5[0]" || instance == "LUT5[1]") {
		if (instance == "LUT5[0]") {
			const string& config = stringify(lut, "6LUT");
			if (mode == "wire") {
				name = escape(stringify(name, "|rt", lut, "6"));
				assert(ipin);
				instPtr->setConfig(config, name, stringify("#LUT:O6=A",ipin));
			}
			else if (mode == "LUT5") {
				name = escape(name);
				// Place holder for fractured LUT
				instPtr->setConfig(config, name, "#LUT:O6=");
				bool b = mNet2InstCfg.insert(make_pair(net, make_tuple(instPtr,config))).second;
				assert(b);

				if (name == "gnd")
					instPtr->setConfig(config, name, "#LUT:O6=0");
				else if (name == "vcc") 
					instPtr->setConfig(config, name, "#LUT:O6=1");
			}
			else throw;

			if (net != "gnd" && net != "vcc") {
				// Activate A if either A or AMUX are O6
				const string& out = parentOutputs.at(stringify("A"));
				const string& outMUX = parentOutputs.at(stringify("AMUX"));
				if (out == "LUT5[0].out[0]->A" || outMUX == "LUT5[0].out[0]->AMUX") {
					addSource(net, instPtr, stringify(lut));
					instPtr->setConfig(stringify(lut, "USED"), "", "0");
				}
			}

			if (!mBels.insert(name).second) {
				cout << config << ":" << name << " duplicated!" << endl;
				throw;
			}
		}
		else if (instance == "LUT5[1]") {
			const string& config = stringify(lut, "5LUT");
			if (mode == "wire") {
				name = escape(stringify(name, "|rt", lut, "5"));
				assert(ipin);
				instPtr->setConfig(config, name, stringify("#LUT:O5=A",ipin));
			}
			else if (mode == "LUT5") {
				name = escape(name);
				instPtr->setConfig(config, name, "#LUT:O5=");
				bool b = mNet2InstCfg.insert(make_pair(net, make_tuple(instPtr,config))).second;
				assert(b);

				if (net == "gnd")
					instPtr->setConfig(config, name, "#LUT:O5=0");
				else if (net == "vcc") 
					instPtr->setConfig(config, name, "#LUT:O5=1");
			}
			else throw;

			if (net != "gnd" && net != "vcc") {
				const string& outMUX = parentOutputs.at(stringify("AMUX"));
				if (outMUX == "LUT5[1].out[0]->AMUX") {
					assert(!instPtr->hasConfig(stringify(lut, "OUTMUX")));
					instPtr->setConfig(stringify(lut, "OUTMUX"), "", "O5");
					addSource(net, instPtr, stringify(lut,"MUX"));
				}
			}

			// If O6 and O5 are both used, fracture O6 by tying A6 to vcc
			const string &cfg6 = stringify(lut,"6LUT");
			if (instPtr->hasConfig(cfg6)) {
				string n, v;
				instPtr->getConfig(cfg6, n, v);
				v.insert(v.rfind('=')+1, "(A6+~A6)*");
				instPtr->setConfig(cfg6, n, v);
				addSink("vcc", instPtr, stringify(lut,"6"));
			}

			if (!mBels.insert(name).second) {
				cout << config << ":" << name << " duplicated!" << endl;
				throw;
			}
		}
		else throw;
	}
	else throw;

	skip(3);
}

void design::parseLUT7(
		InstanceSharedPtr instPtr, 
		const char& lut, 
		const string& bleName,
		const map<string,string>& parentInputs,
		const map<string,string>& parentOutputs
		) {
	assert(lut == 'A' || lut == 'C');
	const string &name = get_attribute("name");
	if (name == "open") {
		return;
	}
	string net = name;

	// HACK: Dummy mask used for now
	instPtr->setConfig(stringify(static_cast<char>(lut+0),"6LUT"), stringify(escape(name), "|lut7a"), "#LUT:O6=A6@A5@A4@A3@A2@A1");
	instPtr->setConfig(stringify(static_cast<char>(lut+1),"6LUT"), stringify(escape(name), "|lut7b"), "#LUT:O6=A6@A5@A4@A3@A2@A1");
	switch (lut) {
		case 'A': instPtr->setConfig("F7AMUX", stringify(escape(name), "|f7"), ""); break;
		case 'C': instPtr->setConfig("F7BMUX", stringify(escape(name), "|f7"), ""); break;
		default: throw;
	}

	const string& out = parentOutputs.at("AMUX");
	// If out is used, or if it out and FF are not used
	if (out == "LUT7[0].F7[0]->AMUX" || parentOutputs.at("AQ") == "open") {
		if (out == "open" && parentOutputs.at("AQ") == "open")
			cout << "parseLUT7(): LUT " << name << " does not exit SLICEL!" << endl;
		instPtr->setConfig(stringify(lut, "OUTMUX"), "", "F7");
		addSource(net, instPtr, stringify(lut, "MUX"));
	}
	else assert(out == "open");

	skip(3);
}

void design::parseLUT8(
		InstanceSharedPtr instPtr, 
		const char& lut, 
		const string& bleName,
		const map<string,string>& parentInputs,
		const map<string,string>& parentOutputs
		) {
	assert(lut == 'B');
	const string &name = get_attribute("name");
	assert(name != "open");
	string net = name;

	// HACK: Dummy mask used for now
	instPtr->setConfig("A6LUT", stringify(escape(name), "|lut8a"), "#LUT:O6=A6@A5@A4@A3@A2@A1");
	instPtr->setConfig("B6LUT", stringify(escape(name), "|lut8b"), "#LUT:O6=A6@A5@A4@A3@A2@A1");
	instPtr->setConfig("C6LUT", stringify(escape(name), "|lut8c"), "#LUT:O6=A6@A5@A4@A3@A2@A1");
	instPtr->setConfig("D6LUT", stringify(escape(name), "|lut8d"), "#LUT:O6=A6@A5@A4@A3@A2@A1");
	instPtr->setConfig("F7AMUX", stringify(escape(name), "|f7a"), "");
	instPtr->setConfig("F7BMUX", stringify(escape(name), "|f7b"), "");
	instPtr->setConfig("F8MUX", stringify(escape(name), "|f8"), "");

	const string& out = parentOutputs.at(stringify(lut, "MUX"));
	// If out is used, or if it out and FF are not used
	if (out == "LUT8[0].F8[0]->BMUX" || parentOutputs.at("BQ") == "open") {
		if (out == "open" && parentOutputs.at("BQ") == "open")
			cout << "parseLUT8(): LUT " << name << " does not exit SLICEL!" << endl;
		instPtr->setConfig("BOUTMUX", "", "F8");
		addSource(net, instPtr, "BMUX");
	}
	else assert(out == "open");

	skip(3);
}


void design::parseXADDER(
		InstanceSharedPtr instPtr, 
		const char& lut, 
		const map<string,string>& parentOutputs
		) {
	const string &name = get_attribute("name");
	const string &mode = get_attribute("mode");
	const string &instance = get_attribute("instance");
	// Blank XADDER
	if (name == "open") {
		assert(mode.empty());
		return;
	}

	// Parse XADDER inputs
	advance(XML_READER_TYPE_ELEMENT, 4, "inputs");
	advance(XML_READER_TYPE_ELEMENT, 5, "port");
	assert(get_attribute("name") == "a_xor_b");
	advance(XML_READER_TYPE_TEXT, 6, "#text");
	const string &a_xor_b = get_value();
	assert(a_xor_b.find_first_of(whitespace) == string::npos);
	skip(5);
	advance(XML_READER_TYPE_ELEMENT, 5, "port");
	assert(get_attribute("name") == "a_and_b");
	advance(XML_READER_TYPE_TEXT, 6, "#text");
	const string &a_and_b = get_value();
	assert(a_and_b.find_first_of(whitespace) == string::npos);
	skip(5);
	advance(XML_READER_TYPE_ELEMENT, 5, "port");
	assert(get_attribute("name") == "cin");
	advance(XML_READER_TYPE_TEXT, 6, "#text");
	const string &cin = get_value();
	assert(cin.find_first_of(whitespace) == string::npos);
	skip(4);

	// Parse XADDER outputs
	advance(XML_READER_TYPE_ELEMENT, 4, "outputs");
	advance(XML_READER_TYPE_ELEMENT, 5, "port");
	assert(get_attribute("name") == "sumout");
	advance(XML_READER_TYPE_TEXT, 6, "#text");
	const string &sumout = get_value();
	assert(sumout.find_first_of(whitespace) == string::npos);
	skip(5);
	advance(XML_READER_TYPE_ELEMENT, 5, "port");
	assert(get_attribute("name") == "cout");
	advance(XML_READER_TYPE_TEXT, 6, "#text");
	const string &carryout = get_value();
	assert(carryout.find_first_of(whitespace) == string::npos);
	skip(4);

	// Ignore XADDER clocks (because there are none)
	skip(XML_READER_TYPE_ELEMENT, 4, "clocks");

	assert(instance == "XADDER[0]");

	assert(a_xor_b == "LUT5[0].out[0]->XADDER_a" /*|| a_xor_b == "open"*/);

	if (a_and_b == "LUT5[1].out[0]->XADDER_b")
		instPtr->setConfig(stringify(lut, "CY0"),
				"",
				"O5");
	else if (a_and_b == "ble6.AX[0]->XADDER_b")
		instPtr->setConfig(stringify(lut, "CY0"),
				"",
				stringify(lut, "X"));
	else assert(a_and_b == "open");

	if (cin == "open") {
		instPtr->setConfig("PRECYINIT", "", "0");
	}

	//if (sumout != "open") {
		const string& outMUX = parentOutputs.at(stringify("AMUX"));
		assert(!instPtr->hasConfig(stringify(lut, "OUTMUX")));
		instPtr->setConfig(stringify(lut, "OUTMUX"), "", "XOR");
		if (outMUX == "XADDER[0].sumout[0]->AMUX") {
			addSource(sumout, instPtr, stringify(lut,"MUX"));
		}
	//}

	if (carryout != "open") {
		const string& outMUX = parentOutputs.at(stringify("AMUX"));
		const string& outCOUT = parentOutputs.at(stringify("COUT"));
		if (lut == 'D') {
			// If DMUX and COUT are both used, activate COUT and rely on route-through
			if (outCOUT == "XADDER[0].cout[0]->COUT" || outMUX == "XADDER[0].cout[0]->AMUX") {
				assert(!instPtr->hasConfig("COUTUSED"));
				instPtr->setConfig("COUTUSED", "", "0");
				addSource(carryout, instPtr, "COUT");
			}
		}
		else if (outMUX == "XADDER[0].cout[0]->AMUX") {
			assert(!instPtr->hasConfig(stringify(lut, "OUTMUX")));
			instPtr->setConfig(stringify(lut, "OUTMUX"), "", "CY");
			addSource(carryout, instPtr, stringify(lut,"MUX"));
		}
	}

	/* Unless [ABCD]MUX is set to sumout (or carryout, which is currently impossible)
	 * then this CARRY4 bel is "non-existent" according to xdl2ncd */
	if (sumout != "open" /*|| carryout != "open"*/)
		instPtr->setConfig("CARRY4", stringify(escape(instPtr->getName()),".CARRY4"), "");

	skip(3);
}


void design::parseFF(
		InstanceSharedPtr instPtr, 
		const char& lut, 
		const map<string,string>& parentOutputs
		) {
	const string &name = get_attribute("name");
	const string &mode = get_attribute("mode");
	const string &instance = get_attribute("instance");
	// Blank FF
	if (name == "open") {
		assert(mode.empty());
		return;
	}

	// Parse FF inputs
	advance(XML_READER_TYPE_ELEMENT, 4, "inputs");
	advance(XML_READER_TYPE_ELEMENT, 5, "port");
	assert(get_attribute("name") == "D");
	advance(XML_READER_TYPE_TEXT, 6, "#text");
	const string &inputs = get_value();
	assert(inputs.find_first_of(whitespace) == string::npos);
	skip(4);

	// Parse FF outputs
	advance(XML_READER_TYPE_ELEMENT, 4, "outputs");
	advance(XML_READER_TYPE_ELEMENT, 5, "port");
	assert(get_attribute("name") == "Q");
	advance(XML_READER_TYPE_TEXT, 6, "#text");
	const string &outputs = get_value();
	assert(outputs == name);
	assert(inputs.find_first_of(whitespace) == string::npos);
	skip(4);

	// Ignore FF clocks (because there is no ambiguity)
	skip(XML_READER_TYPE_ELEMENT, 4, "clocks");

	if (instance == "FF[0]") {
		instPtr->setConfig(stringify(lut, "FF"), escape(name), "#FF");

		string ffmux;
		if (inputs == "LUT6[0].out[0]->FF[0].D" || inputs == "LUT5[0].out[0]->FF[0].D")
			ffmux = "O6";
		else if (inputs == "LUT5[1].out[0]->FF[0].D")
			ffmux = "O5";
		else if (inputs == "ble6.AX[0]->FF[0].D") 
			ffmux = stringify(lut, "X");
		else if (inputs == "XADDER[0].sumout[0]->FF[0].D")
			ffmux = "XOR";
		else if (inputs == "LUT7[0].F7[0]->AFF")
			ffmux = "F7";
		else if (inputs == "LUT8[0].F8[0]->BFF")
			ffmux = "F8";
		else throw;
		
		if (inputs == "ble6.AX[0]->FF[0].D")
			instPtr->setConfig(stringify(lut, "FFMUX"),
					"",
					stringify(lut,"X"));
		else
			instPtr->setConfig(stringify(lut, "FFMUX"), "", ffmux);
		string out;
		if (ffmux == "F8") {
			out = parentOutputs.at("BQ");
			assert(out == "FF[0].Q[0]->BQ");
		}
		else {
			out = parentOutputs.at("AQ");
			assert(out == "FF[0].Q[0]->AQ");
		}
		addSource(name, instPtr, stringify(lut,"Q"));
	}
	else if (instance == "FF[1]") {
		instPtr->setConfig(stringify(lut, "5FF"),
				escape(name),
				"");
		const char *ffmux;
		if (inputs == "LUT5[1].out[0]->FF[1].D")
			ffmux = "IN_A";
		else if (inputs == "ble6.AX[0]->FF[1].D")
			ffmux = "IN_B";
		else throw;
		instPtr->setConfig(stringify(lut, "5FFMUX"),
				"",
				ffmux);
		const string& out = parentOutputs.at(stringify("AMUX"));
		assert(out == "FF[1].Q[0]->AMUX");
		assert(!instPtr->hasConfig(stringify(lut, "OUTMUX")));
		instPtr->setConfig(stringify(lut, "OUTMUX"),
				"",
				stringify(lut,"5Q"));
		addSource(name, instPtr, stringify(lut,"MUX"));

	}
	else throw;

	if (!instPtr->hasConfig("CLKINV"))
		instPtr->setConfig("CLKINV", "", "CLK");

	skip(3);
}

void design::parseBLE6(InstanceSharedPtr instPtr, 
		const char& lut,
		const map<string,string>& parentInputs
		) {
	const string &name = get_attribute("name");
	const string &mode = get_attribute("mode");

	if (name == "open") {
		assert(mode.empty());
		return;
	}
	assert(mode == "O6LUT" || mode == "O6O5LUT" || mode == "O6O5LUT_XADDER");

	// Ignore BLE inputs (because there is no ambiguity)
	skip(XML_READER_TYPE_ELEMENT, 3, "inputs");

	// Parse BLE outputs
	advance(XML_READER_TYPE_ELEMENT, 3, "outputs");
	map<string,string> outputs;
	advance();
	do {
		parseOutputPort(outputs);
		advance();
	} while (get_depth() > 3);

	// Ignore BLE clocks (because there is no ambiguity)
	skip(XML_READER_TYPE_ELEMENT, 3, "clocks");

	int nblock;
	if (mode == "O6O5LUT_XADDER")
		nblock = 5;
	else if (mode == "O6O5LUT")
		nblock = 4;
	else if (mode == "O6LUT")
		nblock = 3;
	else throw;

	for (int i = 0; i < nblock; ++i) {
		advance(XML_READER_TYPE_ELEMENT, 3, "block");
		const string &instance = get_attribute("instance");
		if (instance == "LUT6[0]" || instance == "LUT5[0]" || instance == "LUT5[1]")
			parseLUT6(instPtr, lut, name, parentInputs, outputs);
		else if (instance == "XADDER[0]")
			parseXADDER(instPtr, lut, outputs);
		else if (instance == "FF[0]" || instance == "FF[1]")
			parseFF(instPtr, lut, outputs);
		else throw;
	}

	//cout << get_name() << " " << get_depth() << " " << get_value() << " " << get_attribute("name") << endl;
	skip(2);
}

void design::parseBLE7(InstanceSharedPtr instPtr, 
		const char& lut,
		const map<string,string>& parentInputs
		) {
	const string &name = get_attribute("name");
	const string &mode = get_attribute("mode");

	if (name == "open") {
		assert(mode.empty());
		return;
	}
	assert(mode == "ble7");

	// Ignore BLE inputs (because there is no ambiguity)
	skip(XML_READER_TYPE_ELEMENT, 3, "inputs");

	// Parse BLE outputs
	advance(XML_READER_TYPE_ELEMENT, 3, "outputs");
	map<string,string> outputs;
	advance();
	do {
		parseOutputPort(outputs);
		advance();
	} while (get_depth() > 3);

	// Ignore BLE clocks (because there is no ambiguity)
	skip(XML_READER_TYPE_ELEMENT, 3, "clocks");

	for (int i = 0; i < 2; ++i) {
		advance(XML_READER_TYPE_ELEMENT, 3, "block");
		const string &name = get_attribute("name");
		const string &instance = get_attribute("instance");
		if (instance == "LUT7[0]") {
			parseLUT7(instPtr, lut, name, parentInputs, outputs);
		}
		else if (instance == "FF[0]") {
			parseFF(instPtr, lut, outputs);
		}
		else throw;
	}

	skip(2);
}

void design::parseBLE8(InstanceSharedPtr instPtr, 
		const char& lut,
		const map<string,string>& parentInputs
		) {
	const string &name = get_attribute("name");
	const string &mode = get_attribute("mode");

	if (name == "open") {
		assert(mode.empty());
		return;
	}
	assert(mode == "ble8");

	// Ignore BLE inputs (because there is no ambiguity)
	skip(XML_READER_TYPE_ELEMENT, 3, "inputs");

	// Parse BLE outputs
	advance(XML_READER_TYPE_ELEMENT, 3, "outputs");
	map<string,string> outputs;
	advance();
	do {
		parseOutputPort(outputs);
		advance();
	} while (get_depth() > 3);

	// Ignore BLE clocks (because there is no ambiguity)
	skip(XML_READER_TYPE_ELEMENT, 3, "clocks");

	for (int i = 0; i < 2; ++i) {
		advance(XML_READER_TYPE_ELEMENT, 3, "block");
		const string &instance = get_attribute("instance");
		if (instance == "LUT8[0]") {
			parseLUT8(instPtr, lut, name, parentInputs, outputs);
		}
		else if (instance == "FF[0]") {
			parseFF(instPtr, lut, outputs);
		}
		else throw;
	}

	skip(2);
}


InstanceSharedPtr design::parseBlock(string& blockInstance) {
	const string& name = get_attribute("name");
	const string& instance = get_attribute("instance");
	const string& type = instance.substr(0,instance.find_first_of('['));

	InstanceSharedPtr instPtr = Factory::newInstancePtr(name, 
								type,
								"",
								"");
	bool b = mDesignPtr->addInstance(instPtr);
	assert(b);
	//cout << "Created " << instPtr->getType() << ": '" << instPtr->getName() << "'" << endl;

	// Parse inputs
	advance(XML_READER_TYPE_ELEMENT, 2, "inputs");
	advance();
	do {
		addInputPort(instPtr);
		advance();
	} while (get_depth() > 2);

	// Ignore outputs (because there is no ambiguity)
	skip(XML_READER_TYPE_ELEMENT, 2, "outputs");

	// Ignore clocks (because there are none)
	skip(XML_READER_TYPE_ELEMENT, 2, "clocks");

	// Parse block
	advance(XML_READER_TYPE_ELEMENT, 2, "block");
	assert(get_attribute("name") == name);
	blockInstance = get_attribute("instance");

	// Ignore block inputs
	skip(XML_READER_TYPE_ELEMENT, 3, "inputs");

	// Parse block outputs
	advance(XML_READER_TYPE_ELEMENT, 3, "outputs");
	advance();
	while (get_depth() > 3) {
		addOutputPort(instPtr);
		advance();
	}

	return instPtr;
}

void design::parseIOB(void) {
	const string& mode = get_attribute("mode");
	assert(mode == "ibuf" || mode == "obuf");

	string instance;
	InstanceSharedPtr instPtr = parseBlock(instance);

	if (instance == "ibuf[0]") {
		instPtr->setConfig("IUSED", "", "0");
		instPtr->setConfig("IBUF_LOW_PWR", "", "TRUE");
		instPtr->setConfig("ISTANDARD", "", "LVCMOS25");
	}
	else if (instance == "obuf[0]") {
		instPtr->setConfig("OUSED", "", "0");
		instPtr->setConfig("OSTANDARD", "", "LVCMOS25");
		instPtr->setConfig("DRIVE", "", "12");
		instPtr->setConfig("SLEW", "", "SLOW");
	}
	else throw;
	
	skip(1);
}



void design::parsePS7() {
	const string &mode = get_attribute("mode");
	assert(mode == "PS7");

	string instance;
	InstanceSharedPtr instPtr = parseBlock(instance);
	assert(instance == "PS[0]");

	skip(1);
}


void design::parseBUFG(void) {
	const string &mode = get_attribute("mode");
	assert(mode == "BUFG");

	string instance;
	InstanceSharedPtr instPtr = parseBlock(instance);
	assert(instance == "BUFGCTRL[0]");

	skip(1);
}

void design::parseDSP(void) {
	const string &mode = get_attribute("mode");
	assert(mode == "mult_25x18_COMB");

	string instance;
	InstanceSharedPtr instPtr = parseBlock(instance);
	assert(instance == "mult_25x18[0]");

	instPtr->setConfig("ACASCREG", "", "0");
	instPtr->setConfig("ADREG", "", "0");
	instPtr->setConfig("ALUMODE0INV", "", "ALUMODE0");
	instPtr->setConfig("ALUMODE1INV", "", "ALUMODE1");
	instPtr->setConfig("ALUMODE2INV", "", "ALUMODE2");
	instPtr->setConfig("ALUMODE3INV", "", "ALUMODE3");
	instPtr->setConfig("ALUMODEREG", "", "0");
	instPtr->setConfig("AREG", "", "0");
	addSink("gnd", instPtr, "CEA1");
	addSink("gnd", instPtr, "CEA2");
	instPtr->setConfig("AUTORESET_PATDET", "", "NO_RESET");
	instPtr->setConfig("A_INPUT", "", "DIRECT");

	instPtr->setConfig("BCASCREG", "", "0");
	instPtr->setConfig("BREG", "", "0");
	addSink("gnd", instPtr, "CEB1");
	addSink("gnd", instPtr, "CEB2");
	instPtr->setConfig("B_INPUT", "", "DIRECT");

	instPtr->setConfig("CARRYININV", "", "CARRYIN");
	instPtr->setConfig("CARRYINREG", "", "0");
	instPtr->setConfig("CARRYINSELREG", "", "0");
	instPtr->setConfig("CLKINV", "", "CLK");
	//instPtr->setConfig("CREG", "", "0");
	//addSink("gnd", instPtr, "CEC");

	instPtr->setConfig("DREG", "", "0");

	instPtr->setConfig("INMODE0INV", "", "INMODE0");
	instPtr->setConfig("INMODE1INV", "", "INMODE1");
	instPtr->setConfig("INMODE2INV", "", "INMODE2");
	instPtr->setConfig("INMODE3INV", "", "INMODE3");
	instPtr->setConfig("INMODE4INV", "", "INMODE4");
	addSink("vcc", instPtr, "INMODE2");
	addSink("gnd", instPtr, "INMODE0");
	addSink("gnd", instPtr, "INMODE1");
	addSink("gnd", instPtr, "INMODE3");
	addSink("gnd", instPtr, "INMODE4");
	instPtr->setConfig("INMODEREG", "", "0");

	instPtr->setConfig("MREG", "", "0");
	addSink("gnd", instPtr, "CEM");

	instPtr->setConfig("OPMODE0INV", "", "OPMODE0");
	instPtr->setConfig("OPMODE1INV", "", "OPMODE1");
	instPtr->setConfig("OPMODE2INV", "", "OPMODE2");
	instPtr->setConfig("OPMODE3INV", "", "OPMODE3");
	instPtr->setConfig("OPMODE4INV", "", "OPMODE4");
	instPtr->setConfig("OPMODE5INV", "", "OPMODE5");
	instPtr->setConfig("OPMODE6INV", "", "OPMODE6");
	addSink("vcc", instPtr, "OPMODE0");
	addSink("vcc", instPtr, "OPMODE2");
	addSink("gnd", instPtr, "OPMODE1");
	addSink("gnd", instPtr, "OPMODE3");
	addSink("gnd", instPtr, "OPMODE4");
	addSink("gnd", instPtr, "OPMODE5");
	addSink("gnd", instPtr, "OPMODE6");
	instPtr->setConfig("OPMODEREG", "", "0");

	instPtr->setConfig("PREG", "", "0");
	addSink("gnd", instPtr, "CEP");
	
	instPtr->setConfig("SEL_MASK", "", "MASK");
	instPtr->setConfig("SEL_PATTERN", "", "PATTERN");

	instPtr->setConfig("USE_DPORT", "", "FALSE");
	instPtr->setConfig("USE_MULT", "", "MULTIPLY");
	instPtr->setConfig("USE_PATTERN_DETECT", "", "NO_PATDET");
	instPtr->setConfig("USE_SIMD", "", "ONE48");

	instPtr->setConfig(instPtr->getType(), escape(instPtr->getName()), "");

	stringstream ss;
	ss << hex << 0x3fffffffffff;
	instPtr->setConfig("MASK", "", ss.str());
	ss.str("");
	ss << setfill('0') << setw(12) << 0;
	instPtr->setConfig("PATTERN", "", ss.str());
	
	skip(1);
}

void design::parseRAMB36(void) {
	const string& mode = get_attribute("mode");
	assert(mode.substr(0,9) == "RAMB36E1_");

	const string& name = get_attribute("name");
	const string& instance = get_attribute("instance");
	const string& type = instance.substr(0,instance.find_first_of('['));
	InstanceSharedPtr instPtr = Factory::newInstancePtr(name, 
			type,
			"",
			"");
	bool b = mDesignPtr->addInstance(instPtr);
	assert(b);
	map<string,string> inputs;
	//cout << "Created " << instPtr->getType() << ": '" << instPtr->getName() << "'" << endl;

	static const regex re("RAMB36E1_(\\d+)x(\\d+)_(REGAB|REGA|REGB|COMB)_([sd])p");
	smatch m;
	b = regex_match(mode, m, re);
	assert(b);
	const string& width = m[2];
	const string& reg = m[3];
	const string& ports = m[4];

	// Parse RAMB inputs
	advance(XML_READER_TYPE_ELEMENT, 2, "inputs");
	advance();
	do {
		//addInputPort(instPtr);
		parseInputPort(inputs);
		skip(3);
		advance();
	} while (get_depth() > 2);

	// Ignore RAMB outputs (because there is no ambiguity)
	advance(XML_READER_TYPE_ELEMENT, 2, "outputs");
	map<string,string> pinMap;
	parseRAMB_outputs_top(3, pinMap, "DOADO");
	skip(4);
	parseRAMB_outputs_top(3, pinMap, "DOPADOP");
	skip(4);
	parseRAMB_outputs_top(3, pinMap, "DOBDO");
	skip(4);
	parseRAMB_outputs_top(3, pinMap, "DOPBDOP");
	skip(4);
	skip(2);

	// Parse RAMB clocks
	advance(XML_READER_TYPE_ELEMENT, 2, "clocks");
	advance();
	do {
		addInputPort(instPtr);
		advance();
	} while (get_depth() > 2);

	// Parse memory_slice
	advance(XML_READER_TYPE_ELEMENT, 2, "block");
	assert(get_attribute("mode") == "memory_slice");
	const string& sliceInst = get_attribute("instance");
	// Ignore memory_slice inputs
	skip(XML_READER_TYPE_ELEMENT, 3, "inputs");
	// Parse memory_slice outputs
	advance(XML_READER_TYPE_ELEMENT, 3, "outputs");
	if (ports == "s") {
		parseRAMB_outputs_mid(4, pinMap, "out", sliceInst);
		skip(5);
	}
	else if (ports == "d") {
		parseRAMB_outputs_mid(4, pinMap, "out1", sliceInst);
		skip(5);
		parseRAMB_outputs_mid(4, pinMap, "out2", sliceInst);
		skip(5);
	}
	else throw;
	skip(3);

	// Ignore memory_slice clocks
	skip(XML_READER_TYPE_ELEMENT, 3, "clocks");

	int islice = 0;
	advance();
	do {	
		// Parse memory slice
		check(XML_READER_TYPE_ELEMENT, 3, "block");
		const string &memName = get_attribute("name");
		if (memName != "open") {
			const string &memInst = get_attribute("instance");
			assert(memInst == stringify("memory_slice[",islice,"]"));

			// Skip inputs
			skip(XML_READER_TYPE_ELEMENT, 4, "inputs");

			// Parse outputs
			advance(XML_READER_TYPE_ELEMENT, 4, "outputs");

			if (ports == "s") {
				parseRAMB_outputs_bot(5, pinMap, "out", memInst, instPtr);
				skip(5);
			}
			else if (ports == "d") {
				parseRAMB_outputs_bot(5, pinMap, "out1", memInst, instPtr);
				skip(5);
				parseRAMB_outputs_bot(5, pinMap, "out2", memInst, instPtr);
				skip(5);
			}
			else throw;
			skip(4);
			
			// Skip clocks
			skip(XML_READER_TYPE_ELEMENT, 4, "clocks");

			skip(3);
		}
		++islice;
		advance();
	} while (get_depth() > 2);

	// At its thinnest mode, connect DI[AB] to 1 as well as 0
	if (width == "1") {
		auto it = inputs.find("DIADI0");
		if (it != inputs.end()) {
			const string& net = it->second;
			assert(net != "open");
			bool b = inputs.insert(make_pair("DIADI1", net)).second;
			assert(b);
		}
		it = inputs.find("DIBDI0");
		if (it != inputs.end()) {
			const string& net = it->second;
			assert(net != "open");
			bool b = inputs.insert(make_pair("DIBDI1", net)).second;
			assert(b);
		}
	}
	// At its widest mode, copy ADDRARDADDRL from ADDRBWRADDRL
	else if (width == "72") {
		for (int i = 0; i < 16; ++i) {
			const string& name = stringify("ADDRBWRADDRL",i);
			auto it = inputs.find(name);
			if (it != inputs.end()) {
				const string& net = it->second;
				assert(net != "open");
				bool b = inputs.insert(make_pair(stringify("ADDRARDADDRL",i),net)).second;
				assert(b);
			}
		}
	}

	// Also tie high any unconnected ports (at the LSB end) of the address bus
	bool foundA = false, foundB = false;
	for (int i = 0; i < 16; ++i) {
		if (ports == "d") {
			const string& nameA = stringify("ADDRARDADDRL",i);
			if (!foundA && inputs.count(nameA) == 0) {
				bool b = inputs.insert(make_pair(nameA,"vcc")).second;
				assert(b);
			}
		}
		else foundA = true;
		const string& nameB = stringify("ADDRBWRADDRL",i);
		if (!foundB && inputs.count(nameB) == 0) {
			bool b = inputs.insert(make_pair(nameB,"vcc")).second;
			assert(b);
		}
		else foundB = true;
	}

	// Special case: duplicate WEAL0 and WEBWEL0 across all pins in vector,
	// because VPR only accepts one pin for write enable
	auto it = inputs.find("WEAL0");
	if (it != inputs.end()) {
		const string& name = it->first.substr(0,it->first.length()-1);
		const string& net = it->second;
		assert(net != "open");
		for (int i = 1; i < 4; ++i) {
			bool b = inputs.insert(make_pair(stringify(name, i), net)).second;
			assert(b);
		}
	}
	it = inputs.find("WEBWEL0");
	if (it != inputs.end()) {
		const string& name = it->first.substr(0,it->first.length()-1);
		const string& net = it->second;
		assert(net != "open");
		for (int i = 1; i < 8; ++i) {
			bool b = inputs.insert(make_pair(stringify(name, i), net)).second;
			assert(b);
		}
	}
	// For dual port RAMs, copy the first port clock to the second port
	if (ports == "d") {
		it = inputs.find("CLKARDCLKL");
		if (it != inputs.end()) {
			const string& net = it->second;
			assert(net != "open");
			bool b = inputs.insert(make_pair("CLKBWRCLKL", net)).second;
			assert(b);
		}
	}

	// Add inputs
	for (const auto &it : inputs) {
		const string& name = it.first;
		const string& net = it.second;
		assert(net != "open");
		addSink(net, instPtr, name);
	}

	// Now copy all 'L' into 'U'
	for (const auto &it : inputs) {
		const string& name = it.first;
		// ADDR{ARD,BWR}ADDRL15 does not have 'U' equiv
		if (name == "ADDRARDADDRL15" || name == "ADDRBWRADDRL15"
				|| starts_with(name, "DIADI")
				|| starts_with(name, "DIPADIP")
				|| starts_with(name, "DIBDI")
				|| starts_with(name, "DIPBDIP")
				|| starts_with(name, "DOADO")
				|| starts_with(name, "DOPADOP")
				|| starts_with(name, "DOBDO")
				|| starts_with(name, "DOPBDOP")
		   )
			continue;
		const string& net = it.second;
		assert(net != "open");
		if (name == "RSTRAMARSTRAMLRST")
			addSink(net, instPtr, "RSTRAMARSTRAMU");
		else {
			auto pos = name.length()-1;
			while (name[pos] >= '0' && name[pos] <= '9')
				--pos;
			assert(name[pos] == 'L');
			string newName = name;
			newName[pos] = 'U';
			addSink(net, instPtr, newName);
		}
	}

	instPtr->setConfig("CLKARDCLKLINV", "", "CLKARDCLKL");
	instPtr->setConfig("CLKARDCLKUINV", "", "CLKARDCLKU");
	instPtr->setConfig("CLKBWRCLKLINV", "", "CLKBWRCLKL");
	instPtr->setConfig("CLKBWRCLKUINV", "", "CLKBWRCLKU");

	if (reg == "REGAB") {
		instPtr->setConfig("DOA_REG", "", "1");
		instPtr->setConfig("DOB_REG", "", "1");
	}
	else if (reg == "REGA") {
		instPtr->setConfig("DOA_REG", "", "1");
		instPtr->setConfig("DOB_REG", "", "0");
	}
	else if (reg == "REGB") {
		instPtr->setConfig("DOA_REG", "", "0");
		instPtr->setConfig("DOB_REG", "", "1");
	}
	else if (reg == "COMB") {
		instPtr->setConfig("DOA_REG", "", "0");
		instPtr->setConfig("DOB_REG", "", "0");
	}
	else throw;

	instPtr->setConfig("ENARDENLINV", "", "ENARDENL");
	instPtr->setConfig("ENARDENUINV", "", "ENARDENU");
	addSink("vcc", instPtr, "ENARDENL");
	addSink("vcc", instPtr, "ENARDENU");
	instPtr->setConfig("ENBWRENLINV", "", "ENBWRENL");
	instPtr->setConfig("ENBWRENUINV", "", "ENBWRENU");
	addSink("vcc", instPtr, "ENBWRENL");
	addSink("vcc", instPtr, "ENBWRENU");
	if (starts_with(mDDB.getDeviceName(), "xc7")) {
		instPtr->setConfig("EN_ECC_READ", "", "FALSE");
		instPtr->setConfig("EN_ECC_WRITE", "", "FALSE");
		instPtr->setConfig("EN_PWRGATE", "", "NONE");
	}

	instPtr->setConfig("RAM_EXTENSION_A", "", "NONE");
	instPtr->setConfig("RAM_EXTENSION_B", "", "NONE");
	if (ports == "s")
		instPtr->setConfig("RAM_MODE", "", "SDP");
	else if (ports == "d")
		instPtr->setConfig("RAM_MODE", "", "TDP");
	else throw;

	instPtr->setConfig("RDADDR_COLLISION_HWCONFIG", "", "DELAYED_WRITE");

	instPtr->setConfig("READ_WIDTH_B", "", width);
	if (ports == "d")
		instPtr->setConfig("READ_WIDTH_A", "", width);
	else
		instPtr->setConfig("READ_WIDTH_A", "", "0");

	instPtr->setConfig("REGCLKARDRCLKLINV", "", "REGCLKARDRCLKL");
	instPtr->setConfig("REGCLKARDRCLKUINV", "", "REGCLKARDRCLKU");
	addSink("gnd", instPtr, "REGCLKARDRCLKL");
	addSink("gnd", instPtr, "REGCLKARDRCLKU");
	instPtr->setConfig("REGCLKBLINV", "", "REGCLKBL");
	instPtr->setConfig("REGCLKBUINV", "", "REGCLKBU");
	addSink("gnd", instPtr, "REGCLKBL");
	addSink("gnd", instPtr, "REGCLKBU");
	instPtr->setConfig("RSTRAMARSTRAMLINV", "", "RSTRAMARSTRAML");
	instPtr->setConfig("RSTRAMARSTRAMUINV", "", "RSTRAMARSTRAMU");
	addSink("gnd", instPtr, "RSTRAMARSTRAML");
	addSink("gnd", instPtr, "RSTRAMARSTRAMU");
	instPtr->setConfig("RSTRAMBLINV", "", "RSTRAMBL");
	instPtr->setConfig("RSTRAMBUINV", "", "RSTRAMBU");
	addSink("gnd", instPtr, "RSTRAMBL");
	addSink("gnd", instPtr, "RSTRAMBU");
	instPtr->setConfig("RSTREGARSTREGLINV", "", "RSTREGARSTREGL");
	instPtr->setConfig("RSTREGARSTREGUINV", "", "RSTREGARSTREGU");
	addSink("gnd", instPtr, "RSTREGARSTREGL");
	addSink("gnd", instPtr, "RSTREGARSTREGU");
	instPtr->setConfig("RSTREGBLINV", "", "RSTREGBL");
	instPtr->setConfig("RSTREGBUINV", "", "RSTREGBU");
	addSink("gnd", instPtr, "RSTREGBL");
	addSink("gnd", instPtr, "RSTREGBU");
	instPtr->setConfig("RSTREG_PRIORITY_A", "", "RSTREG");
	addSink("gnd", instPtr, "REGCEAREGCEL");
	addSink("gnd", instPtr, "REGCEAREGCEU");
	instPtr->setConfig("RSTREG_PRIORITY_B", "", "RSTREG");
	addSink("gnd", instPtr, "REGCEBL");
	addSink("gnd", instPtr, "REGCEBU");

	instPtr->setConfig("SAVEDATA", "", "FALSE");

	instPtr->setConfig("WRITE_MODE_A", "", "WRITE_FIRST");
	instPtr->setConfig("WRITE_MODE_B", "", "WRITE_FIRST");

	instPtr->setConfig("WRITE_WIDTH_B", "", width);
	if (ports == "d")
		instPtr->setConfig("WRITE_WIDTH_A", "", width);
	else
		instPtr->setConfig("WRITE_WIDTH_B", "", "0");

	instPtr->setConfig(instPtr->getType(), escape(instPtr->getName()), "");

	for (int i = 0; i < 0x80; ++i) {
		stringstream ss[2];
		ss[0] << setfill('0') << setw(2) << uppercase << hex << i;
		ss[1] << setfill('0') << setw(64) << 0;
		instPtr->setConfig(stringify("INIT_", ss[0].str()), "", ss[1].str());
		if (i < 0x10)
			instPtr->setConfig(stringify("INITP_", ss[0].str()), "", ss[1].str());
	}

	instPtr->setConfig("INIT_A","",  "000000000");
	instPtr->setConfig("INIT_B", "", "000000000");
	instPtr->setConfig("SRVAL_A", "", "000000000");
	instPtr->setConfig("SRVAL_B", "", "000000000");

	skip(1);
}


void design::parseRAMB_outputs_top(
		const int& depth, 
		map<string,string>& pinMap, 
		const string& name) {
	advance(XML_READER_TYPE_ELEMENT, depth, "port");
	assert(get_attribute("name") == name);
	advance(XML_READER_TYPE_TEXT, depth+1, "#text");
	static const regex re("\\s+");
	const string& value = get_value();
	sregex_token_iterator split_first{value.begin(), value.end(), re, -1}, split_last;
	const vector<string> &text = {split_first, split_last};
	int i = 0;
	for (const auto &d : text) {
		if (d != "open") {
			const size_t &p = d.rfind("->");
			pinMap.insert(make_pair(d.substr(0,p), stringify(name, i)));
		}
		++i;
	}
}

void design::parseRAMB_outputs_mid(
		const int& depth,
		map<string,string>& pinMap, 
		const string& name,
		const string& inst) {
	advance(XML_READER_TYPE_ELEMENT, depth, "port");
	assert(get_attribute("name") == name);
	advance(XML_READER_TYPE_TEXT, depth+1, "#text");
	static const regex re("\\s+");
	const string& value = get_value();
	sregex_token_iterator split_first{value.begin(), value.end(), re, -1}, split_last;
	const vector<string> &text = {split_first, split_last};	
	int i = 0;
	for (const auto &d : text) {
		if (d != "open") {
			const size_t &p = d.rfind("->");
			assert(p != string::npos);
			const auto &it = pinMap.find(stringify(inst, ".", name, "[", i, "]"));
			pinMap.insert(make_pair(d.substr(0,p), it->second));
			pinMap.erase(it);
		}
		++i;
	}
}

void design::parseRAMB_outputs_bot(
		const int& depth,
		const map<string,string>& pinMap, 
		const string& name,
		const string& inst,
		InstanceSharedPtr instPtr) {
	advance(XML_READER_TYPE_ELEMENT, depth, "port");
	assert(get_attribute("name") == name);
	advance(XML_READER_TYPE_TEXT, depth+1, "#text");
	static const regex re("\\s+");
	const string& value = get_value();
	sregex_token_iterator split_first{value.begin(), value.end(), re, -1}, split_last;
	const vector<string> &text = {split_first, split_last};	
	int i = 0;
	for (const auto &d : text) {
		if (d != "open") {
			addSource(d, instPtr, pinMap.at(
						stringify(inst, ".", name, "[", i, "]")));
		}
		++i;
	}
}

InstanceSharedPtr design::parseRAMB18(const map<string,string> &parentInputs) {
	const string& name = get_attribute("name");
	const string& mode = get_attribute("mode");
	if (name == "open") {
		assert(mode.empty());
		return InstanceSharedPtr();
	}
	const string& instance = get_attribute("instance");
	const string& type = instance.substr(0,instance.find_first_of('['));
	InstanceSharedPtr instPtr = Factory::newInstancePtr(name, 
			type,
			"",
			"");
	bool b = mDesignPtr->addInstance(instPtr);
	assert(b);
	//cout << "Created " << instPtr->getType() << ": '" << instPtr->getName() << "'" << endl;

	static const regex re("RAMB18E1_(\\d+)x(\\d+)_(REGAB|REGA|REGB|COMB)_([sd])p");
	smatch m;
	b = regex_match(mode, m, re);
	assert(b);
	const string& width = m[2];
	const string& reg = m[3];
	const string& ports = m[4];

	// Ignore RAMB18E1 inputs
	skip(XML_READER_TYPE_ELEMENT, 3, "inputs");

	// Parse RAMB18E1 outputs
	advance(XML_READER_TYPE_ELEMENT, 3, "outputs");
	map<string,string> pinMap;
	parseRAMB_outputs_top(4, pinMap, "DOADO");
	skip(5);
	parseRAMB_outputs_top(4, pinMap, "DOPADOP");
	skip(5);
	parseRAMB_outputs_top(4, pinMap, "DOBDO");
	skip(5);
	parseRAMB_outputs_top(4, pinMap, "DOPBDOP");
	skip(3);
	
	// Ignore RAMB18E1 clocks
	skip(XML_READER_TYPE_ELEMENT, 3, "clocks");

	// Parse memory_slice
	advance(XML_READER_TYPE_ELEMENT, 3, "block");
	assert(get_attribute("mode") == "memory_slice");
	const string& sliceInst = get_attribute("instance");
	// Ignore memory_slice inputs
	skip(XML_READER_TYPE_ELEMENT, 4, "inputs");
	// Parse memory_slice outputs
	advance(XML_READER_TYPE_ELEMENT, 4, "outputs");
	if (ports == "s") {
		parseRAMB_outputs_mid(5, pinMap, "out", sliceInst);
		skip(6);
	}
	else if (ports == "d") {
		parseRAMB_outputs_mid(5, pinMap, "out1", sliceInst);
		skip(6);
		parseRAMB_outputs_mid(5, pinMap, "out2", sliceInst);
		skip(6);
	}
	else throw;
	skip(4);
	
	// Ignore memory_slice clocks
	skip(XML_READER_TYPE_ELEMENT, 4, "clocks");

	int islice = 0;
	advance();
	do {	
		// Parse memory slice
		check(XML_READER_TYPE_ELEMENT, 4, "block");
		const string &memName = get_attribute("name");
		if (memName != "open") {
			const string &memInst = get_attribute("instance");
			assert(memInst == stringify("memory_slice[",islice,"]"));

			// Skip inputs
			skip(XML_READER_TYPE_ELEMENT, 5, "inputs");

			// Parse outputs
			advance(XML_READER_TYPE_ELEMENT, 5, "outputs");

			if (ports == "s") {
				parseRAMB_outputs_bot(6, pinMap, "out", memInst, instPtr);
				skip(6);
			}
			else if (ports == "d") {
				parseRAMB_outputs_bot(6, pinMap, "out1", memInst, instPtr);
				skip(6);
				parseRAMB_outputs_bot(6, pinMap, "out2", memInst, instPtr);
				skip(6);
			}
			else throw;
			skip(5);
			
			// Skip clocks
			skip(XML_READER_TYPE_ELEMENT, 5, "clocks");

			skip(4);
		}
		++islice;
		advance();
	} while (get_depth() > 3);

	// RAMB18E1s always have these tied high
	addSink("vcc", instPtr, "ADDRATIEHIGH0");
	addSink("vcc", instPtr, "ADDRATIEHIGH1");
	addSink("vcc", instPtr, "ADDRBTIEHIGH0");
	addSink("vcc", instPtr, "ADDRBTIEHIGH1");

	// At its widest mode, copy ADDRARDADDR from ADDRBWRADDR
	if (width == "36") {
		for (int i = 0; i < 14; ++i) {
			const string& name = stringify("ADDRBWRADDR",i);
			auto it = parentInputs.find(name);
			if (it != parentInputs.end()) {
				const string& net = it->second;
				assert(net != "open");
				addSink(net, instPtr, stringify("ADDRARDADDR",i));
			}
		}
	}
	// Also tie high any unconnected ports (at the LSB end) of the address bus
	bool foundA = false, foundB = false;
	for (int i = 0; i < 14; ++i) {
		if (ports == "d") {
			const string& nameA = stringify("ADDRARDADDR",i);
			if (!foundA && parentInputs.count(nameA) == 0)
				addSink("vcc", instPtr, nameA);
			else foundA = true;
		}
		const string& nameB = stringify("ADDRBWRADDR",i);
		if (!foundB && parentInputs.count(nameB) == 0)
			addSink("vcc", instPtr, nameB);
		else foundB = true;
	}

	// Special case: duplicate WEA0 and WEBWE0 across all pins in vector,
	// because VPR only accepts one pin for write enable
	auto it = parentInputs.find("WEA0");
	if (it != parentInputs.end()) {
		for (int i = 1; i < 4; ++i) 
			addSink(it->second, instPtr, stringify(
						it->first.substr(0,it->first.length()-1),
						i));
	}
	it = parentInputs.find("WEBWE0");
	if (it != parentInputs.end()) {
		for (int i = 1; i < 4; ++i) 
			addSink(it->second, instPtr, stringify(
						it->first.substr(0,it->first.length()-1),
						i));
	}
	// WEBWE[7:4] are always gnd for RAMB18E1
	for (int i = 4; i < 8; ++i) 
		addSink("gnd", instPtr, stringify("WEBWE", i));

	// For dual port RAMs, copy the first port clock to the second port
	if (ports == "d") {
		it = parentInputs.find("CLKARDCLK");
		assert(it->second != "open");
		addSink(it->second, instPtr, "CLKBWRCLK");
	}

	// Add inputs from parents into this instance
	for (const auto &it : parentInputs) {
		const string& name = it.first;
		const string& net = it.second;
		assert(net != "open");
		addSink(net, instPtr, name);
	}

	instPtr->setConfig("CLKARDCLKINV", "", "CLKARDCLK");
	instPtr->setConfig("CLKBWRCLKINV", "", "CLKBWRCLK");

	if (reg == "REGAB") {
		instPtr->setConfig("DOA_REG", "", "1");
		instPtr->setConfig("DOB_REG", "", "1");
	}
	else if (reg == "REGA") {
		instPtr->setConfig("DOA_REG", "", "1");
		instPtr->setConfig("DOB_REG", "", "0");
	}
	else if (reg == "REGB") {
		instPtr->setConfig("DOA_REG", "", "0");
		instPtr->setConfig("DOB_REG", "", "1");
	}
	else if (reg == "COMB") {
		instPtr->setConfig("DOA_REG", "", "0");
		instPtr->setConfig("DOB_REG", "", "0");
	}
	else throw;

	instPtr->setConfig("ENARDENINV", "", "ENARDEN");
	addSink("vcc", instPtr, "ENARDEN");
	instPtr->setConfig("ENBWRENINV", "", "ENBWREN");
	addSink("vcc", instPtr, "ENBWREN");
	if (starts_with(mDDB.getDeviceName(), "xc7")) {
		instPtr->setConfig("EN_PWRGATE", "", "NONE");
	}

	if (ports == "s")
		instPtr->setConfig("RAM_MODE", "", "SDP");
	else if (ports == "d")
		instPtr->setConfig("RAM_MODE", "", "TDP");
	else throw;

	instPtr->setConfig("RDADDR_COLLISION_HWCONFIG", "", "DELAYED_WRITE");

	instPtr->setConfig("READ_WIDTH_B", "", width);
	if (ports == "d")
		instPtr->setConfig("READ_WIDTH_A", "", width);
	else
		instPtr->setConfig("READ_WIDTH_A", "", "0");

	instPtr->setConfig("REGCLKARDRCLKINV", "", "REGCLKARDRCLK");
	addSink("gnd", instPtr, "REGCLKARDRCLK");
	instPtr->setConfig("REGCLKBINV", "", "REGCLKB");
	addSink("gnd", instPtr, "REGCLKB");
	instPtr->setConfig("RSTRAMARSTRAMINV", "", "RSTRAMARSTRAM");
	addSink("gnd", instPtr, "RSTRAMARSTRAM");
	instPtr->setConfig("RSTRAMBINV", "", "RSTRAMB");
	addSink("gnd", instPtr, "RSTRAMB");
	instPtr->setConfig("RSTREGARSTREGINV", "", "RSTREGARSTREG");
	addSink("gnd", instPtr, "RSTREGARSTREG");
	instPtr->setConfig("RSTREGBINV", "", "RSTREGB");
	addSink("gnd", instPtr, "RSTREGB");
	instPtr->setConfig("RSTREG_PRIORITY_A", "", "RSTREG");
	addSink("gnd", instPtr, "REGCEAREGCE");
	instPtr->setConfig("RSTREG_PRIORITY_B", "", "RSTREG");
	addSink("gnd", instPtr, "REGCEB");

	instPtr->setConfig("SAVEDATA", "", "FALSE");

	instPtr->setConfig("WRITE_MODE_A", "", "WRITE_FIRST");
	instPtr->setConfig("WRITE_MODE_B", "", "WRITE_FIRST");

	instPtr->setConfig("WRITE_WIDTH_B", "", width);
	if (ports == "d")
		instPtr->setConfig("WRITE_WIDTH_A", "", width);
	else
		instPtr->setConfig("WRITE_WIDTH_A", "", "0");

	instPtr->setConfig(instPtr->getType(), escape(instPtr->getName()), "");

	for (int i = 0; i < 0x40; ++i) {
		stringstream ss[2];
		ss[0] << setfill('0') << setw(2) << uppercase << hex << i;
		ss[1] << setfill('0') << setw(64) << 0;
		instPtr->setConfig(stringify("INIT_", ss[0].str()), "", ss[1].str());
		if (i < 0x8)
			instPtr->setConfig(stringify("INITP_", ss[0].str()), "", ss[1].str());
	}

	instPtr->setConfig("INIT_A","",  "000000000");
	instPtr->setConfig("INIT_B", "", "000000000");
	instPtr->setConfig("SRVAL_A", "", "000000000");
	instPtr->setConfig("SRVAL_B", "", "000000000");

	skip(2);

	return instPtr;
}

void design::parseRAMB18x2(void) {
	const string &name = get_attribute("name");
	const string &mode = get_attribute("mode");
	assert(mode == "RAMB18E1x2");
	map<string,string> inputs[2];

	// Parse RAMB inputs
	advance(XML_READER_TYPE_ELEMENT, 2, "inputs");
	advance();
	do {
		const string& portName = get_attribute("name");
		if (starts_with(portName, "s0_"))
			parseInputPort(inputs[0]);
		else
			parseInputPort(inputs[1]);
		skip(3);
		advance();
	} while (get_depth() > 2);

	// Ignore RAMB outputs (because there is no ambiguity)
	skip(XML_READER_TYPE_ELEMENT, 2, "outputs");

	// Parse RAMB clocks
	advance(XML_READER_TYPE_ELEMENT, 2, "clocks");
	advance();
	do {
		const string& portName = get_attribute("name");
		if (starts_with(portName, "s0_"))
			parseInputPort(inputs[0]);
		else
			parseInputPort(inputs[1]);
		skip(3);
		advance();
	} while (get_depth() > 2);
	
	// Parse block
	advance(XML_READER_TYPE_ELEMENT, 2, "block");
	assert(get_attribute("instance") == "RAMB18E1[0]");
	InstanceSharedPtr primaryRAMB = parseRAMB18(inputs[0]);

	// Parse block
	advance(XML_READER_TYPE_ELEMENT, 2, "block");
	assert(get_attribute("instance") == "RAMB18E1[1]");
	InstanceSharedPtr secondaryRAMB = parseRAMB18(inputs[1]);

	if (!primaryRAMB || primaryRAMB->getName() != name) {
		assert(!secondaryRAMB || secondaryRAMB->getName() == name);
		swap(primaryRAMB,secondaryRAMB);
	}
	bool b = mSecondaryRAMB18.insert(make_pair(primaryRAMB,secondaryRAMB)).second;
	assert(b);

	skip(1);
}

void design::parseSLICEL(void) {
	const string &name = get_attribute("name");
	const string &instance = get_attribute("instance");
	const string &type = instance.substr(0,instance.find_first_of('['));
	const string &mode = get_attribute("mode");
	InstanceSharedPtr instPtr = Factory::newInstancePtr(name, 
								type,
								"",
								"");
	bool b = mDesignPtr->addInstance(instPtr);
	assert(b);
	//cout << "Created " << instPtr->getType() << ": '" << instPtr->getName() << "'" << endl;

	// Parse SLICE inputs
	map<string,string> inputs;
	advance(XML_READER_TYPE_ELEMENT, 2, "inputs");
	advance();
	do {
		//addInputPort(instPtr);
		parseInputPort(inputs);
		skip(3);
		advance();
	} while (get_depth() > 2);
	for (const auto it : inputs) {
		const string& pin = it.first;
		const string& net = it.second;
		addSink(net, instPtr, pin);
	}

	// Ignore SLICE outputs (because there is no ambiguity)
	skip(XML_READER_TYPE_ELEMENT, 2, "outputs");

	// Parse SLICE clocks
	advance(XML_READER_TYPE_ELEMENT, 2, "clocks");
	advance();
	do {
		addInputPort(instPtr);
		advance();
	} while (get_depth() > 2);

	if (mode == "LUT6x4" || mode == "SLICEL") {
		for (int i = 0; i < 4; ++i) {
			advance(XML_READER_TYPE_ELEMENT, 2, "block");
			parseBLE6(instPtr, 'A'+i, inputs);
		}
	}
	else if (mode == "LUT7x2") {
		for (int i = 0; i < 2; ++i) {
			advance(XML_READER_TYPE_ELEMENT, 2, "block");
			parseBLE7(instPtr, 'A'+i*2, inputs);
		}
	}
	else if (mode == "LUT8") {
		advance(XML_READER_TYPE_ELEMENT, 2, "block");
		parseBLE8(instPtr, 'B', inputs);
	}
	else throw;

	skip(1);
}


void design::addSink(const string &net,
			InstanceSharedPtr instPtr,
			const string &pin) {
	assert(net != "open");
	Circuit::NetSharedPtrIterator netPtrIt = mDesignPtr->findNet(net);
	NetSharedPtr netPtr;
	if (netPtrIt == mDesignPtr->netsEnd()) {
		ENetType type = eNetTypeNormal;
		if (net == "vcc") type = eNetTypePower;
		else if (net == "gnd") type = eNetTypeGround;
		netPtr = Factory::newNetPtr(net, type);
		mDesignPtr->addNet(netPtr);
	}
	else
		netPtr = *netPtrIt;

	InstancePinSharedPtr instPinPtr = Factory::newInstancePinPtr(instPtr, pin);
	assert(!netPtr->containsSink(instPinPtr));
	netPtr->addSink(instPinPtr);
}

void design::addSource(const string &net,
			InstanceSharedPtr instPtr,
			const string &pin) {
	assert(net != "open");
	if (net == "vcc" || net == "gnd")
		return;
	Circuit::NetSharedPtrIterator netPtrIt = mDesignPtr->findNet(net);
	NetSharedPtr netPtr;
	if (netPtrIt == mDesignPtr->netsEnd()) {
		netPtr = Factory::newNetPtr(net);
		mDesignPtr->addNet(netPtr);
	}
	else
		netPtr = *netPtrIt;

	InstancePinSharedPtr instPinPtr = Factory::newInstancePinPtr(instPtr, pin);
	assert(!netPtr->hasAnySources());
	netPtr->addSource(instPinPtr);
}


	int
main(int argc, char* argv[])
{
	if (argc != 3) {
		cerr << argv[0] << " <arch> <circuit>" << endl;
		return -1;
	}
	const boost::regex re("(xc.*)((ff|ffg|clg)\\d+)(_.*)?");
	boost::cmatch what;
	bool b = boost::regex_match(argv[1], what, re);
	if (!b) {
		cerr << argv[0] << " <arch> <circuit>" << endl;
		return -1;
	}
	const string device(what[1]);
	const string package(what[2]);
	const char *circuit = argv[2];
	try
	{
		design d(device, package, "-1");
		d(circuit);

		const string &xdl = stringify(circuit,".xdl");
		ofstream xdlExport(basename(xdl.c_str()));
		d.writeXdl(xdlExport);
		xdlExport.close();
	}
	catch(const exception& e)
	{
		cout << "Exception caught: " << e.what() << endl;
	}

	cout << "Success!" << endl;
	return 0;
}
