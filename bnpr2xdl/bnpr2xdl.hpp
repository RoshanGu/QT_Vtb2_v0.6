#include <fstream>
//#include <regex>
#include <boost/regex.hpp>
using boost::regex;
using boost::smatch;
using boost::sregex_token_iterator;

//#include <libxml++/libxml++.h>
//#include <libxml++/parsers/textreader.h>
#include <libxml/xmlreader.h>

#include "torc/physical/Factory.hpp"
#include "torc/physical/XdlExporter.hpp"
#include "torc/Architecture.hpp"
//#include "torc/Common.hpp"

using namespace std;
using namespace torc::physical;
//using namespace torc::common;
//using namespace torc::architecture;
//using namespace torc::architecture::Xilinx;

namespace ta = torc::architecture;
namespace tax = torc::architecture::xilinx;

using ta::DDB;
using ta::Tiles;
using ta::Sites;
using ta::WireUsage;
using ta::Tilewire;
using ta::TilewireVector;
using ta::Arc;
using ta::ExtendedWireInfo;
using tax::TileIndex;

template <class T>
std::string stringify(T const &t) {
    // This could use lexical_cast, but for now we'll keep dependencies to a minimum
    std::stringstream b;
    b << t;
    return b.str();
}

template<typename T, typename... Args>
std::string stringify(T arg, const Args&... args) {
    return stringify(arg) + stringify(args...);
}

class design {
	public:
		design(		const string& device,
				const string& package,
				const string& speed,
				const string& version);

		void operator()(const string &circuit);
		void writeXdl(ostream &fs) {
			XdlExporter fileExporter(fs);
			fileExporter(mDesignPtr);
		}

	protected:
		const string mDevice, mPackage, mSpeed, mVersion;
		DDB mDDB;
		DesignSharedPtr mDesignPtr;
		//xmlpp::TextReader *mXml;
		xmlTextReaderPtr mXml;
		map<string, tuple<InstanceSharedPtr, string> > mNet2InstCfg;
		map<InstanceSharedPtr,InstanceSharedPtr> mSecondaryRAMB18;
		set<string> mBels;

		static const string whitespace;

		static bool siteComp(const string &l, const string &r) {
			static const regex re("(RAMB18|RAMB36)?.*_X(\\d+)Y\\d+");
			static smatch ml, mr;
			const bool &bl = regex_match(l, ml, re);
			assert(bl);
			const bool &br = regex_match(r, mr, re);
			assert(br);
			// In a BRAM tile, RAMB18 goes first
			return (ml[1] < mr[1]) 
				|| (boost::lexical_cast<int>(ml[2]) < boost::lexical_cast<int>(mr[2]));
		}

		void include(void);
		void parseNet(const string &net);
		void parsePlace(const string &place);
		void genClbTile2Sites(map<TileIndex, vector<string> > &tile2sites) {
			const Sites& sites = mDDB.getSites();
			const Tiles& tiles = mDDB.getTiles();
			for (const auto& s : sites.getSites()) {
				const auto& ti = s.getTileIndex();
				const auto& tInfo = tiles.getTileInfo(ti);
				const std::string& tileTypeName = tiles.getTileTypeName(tInfo.getTypeIndex());
				if (boost::starts_with(tileTypeName, "CLBLL") || boost::starts_with(tileTypeName, "CLBLM")
						|| boost::starts_with(tileTypeName, "DSP")
						|| boost::starts_with(tileTypeName, "BRAM")
					) {
					const auto& sn = s.getName();
					const auto& it = tile2sites.find(ti);
					if (it == tile2sites.end())
						tile2sites.insert(make_pair(ti, vector<string>({ sn })));
					else
						it->second.push_back(sn);
				}
			}

			for (auto& s : tile2sites) {
				vector<string> &v = s.second;
				sort(v.begin(), v.end(), siteComp);
			}
		}
		void readIob2Site(map<tuple<int,int,int>, string> &iob2site) {
			static const regex re("IOB_X(\\d+)Y(\\d+)");
			smatch m;

			ifstream fs(stringify(mDesignPtr->getDevice(), mDesignPtr->getPackage(), ".pkg"));
			assert(fs);
			fs.ignore(1024, '\n');
			string what, tile, site;
			while (fs >> what >> tile >> site) {
				if (what == "pin") {
					if (regex_match(tile, m, re)) {
						const int x = boost::lexical_cast<int>(m[1]);
						const int y = boost::lexical_cast<int>(m[2]);
						const int z = 1 - (y % 2);
						//cout << "iob2site: " << x << "," << y-(1-z) << "," << z << endl;
						const bool &b = iob2site.insert(make_pair(make_tuple(x,y-z,z), site)).second;
						assert(b);
					}
				}
			}
			fs.close();
		}
		void parseBlif(const string& blif);
		void parseRoute(const string& route);
		void readInode2Tw(vector<Tilewire>& inode2tw, int &inodeOffset);
		void genTileLut2InstPin(NetSharedPtr netPtr, map< tuple<TileIndex,string>, vector<InstancePinSharedPtr> >& tileLut2InstPin) {
			for (auto it = netPtr->sinksBegin(), ie = netPtr->sinksEnd(); it != ie; ++it) {
				const auto& instPinPtr = ta::InstancePin::physicalToArchitecture(*it);
				instPinPtr->updateTilewire(mDDB);
				const auto& ewi = ExtendedWireInfo(mDDB, instPinPtr->getTilewire());
				const std::string tileTypeName(ewi.mTileTypeName);
				const std::string wireName(ewi.mWireName);
				if (isClb(ewi.mTileTypeName) && wireName.back() >= '1' && wireName.back() <= '6') {
					const auto& k = make_tuple(ewi.mTileIndex, wireName.substr(6, wireName.length()-6-1));
					auto jt = tileLut2InstPin.find(k);
					if (jt == tileLut2InstPin.end())
						jt = tileLut2InstPin.insert(make_pair(k, vector<InstancePinSharedPtr>() )).first;
					jt->second.push_back(*it);
				}
			}
		}
		void genIntTile2Site(map<string, string> &tile2site) {
			const Sites& sites = mDDB.getSites();
			const Tiles& tiles = mDDB.getTiles();
			for (const auto& s : sites.getSites()) {
				const auto& ti = s.getTileIndex();
				const auto& tInfo = tiles.getTileInfo(ti);
				const std::string& tileTypeName = tiles.getTileTypeName(tInfo.getTypeIndex());
				if (boost::starts_with(tileTypeName, "INT")) {
					const auto& sn = s.getName();
					const auto& tn = tInfo.getName();
					bool b = tile2site.insert(make_pair(tn, sn)).second;
					assert(b);
				}
			}
		}


		Arc findArc(const Tilewire& twSrc, const Tilewire& twSnk);
		inline bool isClb(const std::string& tileTypeName) {
			return boost::starts_with(tileTypeName, "CLBLL") || boost::starts_with(tileTypeName, "CLBLM");
		}

		InstanceSharedPtr findOrCreateTieoff(const map<string, string> &tile2site, int x, int y) {
			auto it = tile2site.find(stringify("INT_X",x,"Y",y));
			if (it == tile2site.end())
				it = tile2site.find(stringify("INT_L_X",x,"Y",y));
			if (it == tile2site.end())
				it = tile2site.find(stringify("INT_R_X",x,"Y",y));
			assert(it != tile2site.end());
			const string& tile = it->first;
			const string& site = it->second;
			const string& instName = stringify("XDL_DUMMY_", tile, "_", site);
			InstanceSharedPtr instPtr;
			Design::InstanceSharedPtrIterator jt = mDesignPtr->findInstance(instName);
			if (jt != mDesignPtr->instancesEnd()) {
				instPtr = *jt;
			}
			else {
				instPtr = Factory::newInstancePtr(instName, "TIEOFF", tile, site);
				instPtr->setConfig("_NO_USER_LOGIC","","");
				bool b = mDesignPtr->addInstance(instPtr);
				assert(b);
			}
			return instPtr;
		}

		void parsePS7(void);
		void parseBUFG(void);
		void parseIOB(void);
		void parseSLICEL(void);
		void parseDSP(void);
		void parseRAMB36(void);
		void parseRAMB18x2(void);
		InstanceSharedPtr parseRAMB18(const map<string,string>& parentInputs);
		void parseRAMB_outputs_top(
				const int& depth, 
				map<string,string>& pinMap, 
				const string& name);
		void parseRAMB_outputs_mid(
				const int& depth,
				map<string,string>& pinMap, 
				const string& name, 
				const string& inst);
		void parseRAMB_outputs_bot(
				const int& depth,
				const map<string,string>& pinMap, 
				const string& name, 
				const string& inst, 
				InstanceSharedPtr instPtr);
		InstanceSharedPtr parseBlock(string& blockInstance);

		inline bool read(void) {
			//return mXml->read();
			return xmlTextReaderRead(mXml) == 1;
		}

		inline void advance(void) {
			assert(read());
			//while (get_node_type() == xmlpp::TextReader::SignificantWhitespace)
			while (get_node_type() == XML_READER_TYPE_SIGNIFICANT_WHITESPACE)
				assert(read());
		}

		inline void check(
				//const xmlpp::TextReader::xmlNodeType &type, 
				const int& type, 
				const int& depth,
				const char *name) {
			assert(get_node_type() == type);
			assert(get_depth() == depth);
			assert(get_name() == name);
		}

		inline void advance(
				//const xmlpp::TextReader::xmlNodeType &type, 
				const int& type, 
				const int& depth,
				const char *name) {
			advance();
			return check(type, depth, name);
		}

		inline void skip(
				//const xmlpp::TextReader::xmlNodeType &type, 
				const int& type, 
				const int& depth,
				const char *name) {
			advance(type, depth, name);
			skip(depth);
		}

		inline void skip(const int &depth) {
			//assert(get_depth() > depth);
			do {
				advance();
			} while(get_depth() > depth);
		}

		inline void skip(void) {
			const int &depth = get_depth();
			skip(depth);
		}

		//inline xmlpp::TextReader::xmlNodeType get_node_type(void) {
		inline int get_node_type(void) {
			//return mXml->get_node_type();
			return xmlTextReaderNodeType(mXml);
		}

		inline string get_name(void) {
			//return mXml->get_name();
			xmlChar* s = xmlTextReaderName(mXml);
			string str(reinterpret_cast<char*>(s));
			xmlFree(s);
			return str;
		}

		inline int get_depth(void) {
			//return mXml->get_depth();
			return xmlTextReaderDepth(mXml);
		}

		// General tool to strip spaces from both ends of a string
		// from Bruce Eckel's "Thinking in C++" book
		inline string trim(const string& s) {
			if(s.length() == 0)
				return s;
			size_t beg = s.find_first_not_of(whitespace);
			size_t end = s.find_last_not_of(whitespace);
			if(beg == string::npos) // No non-spaces
				return "";
			return string(s, beg, end - beg + 1);
		}

		inline string get_value(void) {
			//return trim(mXml->get_value());
			xmlChar* s = xmlTextReaderValue(mXml);
			if (!s) return "";
			string str(reinterpret_cast<char*>(s));
			xmlFree(s);
			return trim(str);
		}

		inline string get_attribute(const char* a) {
			//return mXml->get_attribute(a);
			xmlChar* s = xmlTextReaderGetAttribute(mXml, reinterpret_cast<const xmlChar*>(a));
			if (!s) return "";
			string str(reinterpret_cast<char*>(s));
			xmlFree(s);
			return str;
		}

		void addSink(const string &net,
				InstanceSharedPtr instPtr,
				const string &pin);
		void addSource(const string &net,
				InstanceSharedPtr instPtr,
				const string &pin);

		void addInputPort(InstanceSharedPtr instPtr);
		void addOutputPort(InstanceSharedPtr instPtr);
		void parseInputPort(map<string,string>& inputs);
		void parseOutputPort(map<string,string>& outputs);
		void parseBLE6(InstanceSharedPtr instPtr, 
				const char& lut,
				const map<string,string>& parentInputs);
		void parseBLE7(InstanceSharedPtr instPtr, 
				const char& lut,
				const map<string,string>& parentInputs);
		void parseBLE8(InstanceSharedPtr instPtr, 
				const char& lut,
				const map<string,string>& parentInputs);
		void parseLUT6(InstanceSharedPtr instPtr, 
				const char& lut, 
				const string& bleName,
				const map<string,string>& parentInputs,
				const map<string,string>& parentOutputs);
		void parseLUT7(InstanceSharedPtr instPtr, 
				const char& lut, 
				const string& bleName,
				const map<string,string>& parentInputs,
				const map<string,string>& parentOutputs);
		void parseLUT8(InstanceSharedPtr instPtr, 
				const char& lut, 
				const string& bleName,
				const map<string,string>& parentInputs,
				const map<string,string>& parentOutputs);


		void parseFF(InstanceSharedPtr instPtr, 
				const char& lut, 
				const map<string,string>& parentOutputs);
		void parseXADDER(InstanceSharedPtr instPtr, 
				const char& lut, 
				const map<string,string>& parentOutputs);
};
