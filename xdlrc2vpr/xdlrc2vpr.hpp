// Torc - Copyright 2011 University of Southern California.  All Rights Reserved.
// $HeadURL: https://svn.east.isi.edu/torc/trunk/src/torc/architecture/DDB.hpp $
// $Id: DDB.hpp 1436 2013-05-06 20:47:54Z nsteiner $

// This program is free software: you can redistribute it and/or modify it under the terms of the 
// GNU General Public License as published by the Free Software Foundation, either version 3 of the 
// License, or (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See 
// the GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License along with this program.  If 
// not, see <http://www.gnu.org/licenses/>.

// *******************************************************************************
// EH: This file is an adaptation of ../torc/src/torc/architecture/VprExporter.hpp
// *******************************************************************************

#ifndef TORC_ARCHITECTURE_VPREXPORTER_HPP
#define TORC_ARCHITECTURE_VPREXPORTER_HPP

#include <iostream>
#include <fstream>
#include "torc/architecture/DDB.hpp"
#include "torc/architecture/XilinxDatabaseTypes.hpp"
#include "torc/architecture/OutputStreamHelpers.hpp"
#include "boost/lexical_cast.hpp"
#include "boost/algorithm/string.hpp"
#include "vpr_types.h"

namespace torc {
namespace architecture {

	class Xdlrc2Vpr {
	protected:
		typedef std::fstream fstream; 
		typedef std::string string; 
		typedef std::vector<string> StringVector; 
		typedef std::vector<uint32_t> Uint32Vector; 
		typedef xilinx::WireFlags WireFlags; 
		typedef xilinx::WireCount WireCount; 
		typedef xilinx::WireIndex WireIndex; 
		typedef xilinx::TileCount TileCount; 
		typedef xilinx::TileIndex TileIndex; 
		typedef xilinx::TileCol TileCol; 
		typedef xilinx::TileRow TileRow; 
		typedef xilinx::TileTypeIndex TileTypeIndex; 
		
		DDB& mDDB;
		const Tiles& mTiles;
		const Segments& mSegments;
		const std::string mPackageName;

		std::vector<t_rr_node> rr_node;
		int num_nodes;
		std::vector<char> edge_switch;
		std::map<int, std::tuple< std::vector<int>, std::vector<short> > > inode2EdgeSwitch;
		std::vector<int> type;
		std::map<std::tuple<int,int,int,t_rr_type,int>, int> ptc2inode;
		void loadVpr(const string &fn);

		std::multimap<std::string, std::tuple<int,int,e_rr_type> > name2ptc;
		void loadPtc2Name(const string &fn);

		inline std::tuple<TileRow,TileCol> ewi2rc(const ExtendedWireInfo &ewi) 
		{
			static const boost::regex re("(.*)_X(\\d+)Y(\\d+)");
			static boost::match_results<const char*> what;
			boost::regex_match(ewi.mTileName, what, re);
			const std::string tile(what[1]);
			TileRow row(boost::lexical_cast<int>(what[3]));
			TileCol col(boost::lexical_cast<int>(what[2]));

			// Add one to the Y coordinate to align it with VPR
			// (Row 0 in VPR land is the bottom row of I/Os,
			// which are empty in VTB)

			row += 1;

			return std::make_tuple(row,col);
		}

		inline bool include(const ExtendedWireInfo &ewi)
		{
			TileRow row;
			TileCol col;

			if (!WireInfo::isOutput(ewi.mWireFlags) && !WireInfo::isInput(ewi.mWireFlags)) {
				TilewireVector sources, sinks;
				mDDB.expandTilewireSources(Tilewire(ewi.mTileIndex, ewi.mWireIndex), sources);
				mDDB.expandTilewireSinks(Tilewire(ewi.mTileIndex, ewi.mWireIndex), sinks);
				if (sources.size() == 0 && sinks.size() == 0)
					return false;
			}

			std::tie(row,col) = ewi2rc(ewi);

			if (row < 1 || row > mHeight+1) return false;
			if (col < 0 || col > mWidth) return false;

			const char *type = ewi.mTileTypeName;

			/* Ignore HCLK_(INNER|OUTER)_IOI tiles
			 * to prevent IOBs going directly onto the
			 * BUFG network, and blowing up runtime... */
			static const boost::regex re_hclk_ioi("HCLK_(INNER_|OUTER_)IOI3?");
			if (boost::regex_match(type, re_hclk_ioi))
				return false;

			if (strncmp(type, "CLBLL", 5) && strncmp(type, "CLBLM", 5) && strncmp(type, "INT", 3)
				&& strncmp(type, "BRAM", 4) && strncmp(type, "DSP", 3) 
				&& strncmp(type, "LIOB", 4) && strncmp(type, "LIOB_FT", 7) && strncmp(type, "RIOB", 4) 
				&& strcmp(type, "IOI") && strncmp(type, "LIOI", 4) && strncmp(type, "RIOI", 4)
				&& strncmp(type, "HCLK", 4) && strncmp(type, "CMT", 3) && strncmp(type, "CLK", 3)
				&& strncmp(type, "PSS", 3)
				)
			{
				return false;
			}
			const char *wire = ewi.mWireName;
			if (strcmp(wire, "HCLK_CLB_M_COUT") == 0
				|| strcmp(wire, "HCLK_CLB_L_COUT") == 0
				// Skip COUT -> CIN tilewire on segment
				// (because it's in the tile below and messes up the CIN IPIN)
				|| strcmp(wire, "CLBLM_L_COUT_N") == 0
				|| strcmp(wire, "CLBLM_M_COUT_N") == 0
				|| strcmp(wire, "CLBLL_L_COUT_N") == 0
				|| strcmp(wire, "CLBLL_LL_COUT_N") == 0
				//std::cerr << "Wrong wire: " << ewi << std::endl;

				// Because for some reason, this wire (0@1488) 
				// is on the same segment as EE2BEG0 (36@2605)
				|| strcmp(wire, "CMT_FIFO_EEA2A0_0") == 0
			   ) {
				return false;
			}

			return true;
		}

		static const boost::regex re_clk;
		static const boost::regex re_124;
		static const boost::regex re_L;
		static const boost::regex re_CLB;
		static const boost::regex re_BYP;
		boost::regex re_BYP_B;
		static const boost::regex re_FAN;
		static const boost::regex re_BOUNCE_NS;
		static const boost::regex re_BRAM_DI;
		static const boost::regex re_BRAM_AD;
		static const boost::regex re_BRAM_WE;
		static const boost::regex re_DSP;

		int getIswitch(const ExtendedWireInfo &ewiSrc, const Tilewire &twSnk);

		const std::string mPostfix;
		const int mWidth, mHeight;

		string getStitchWirename(const Tilewire &tw);
		short getStitchY(const ExtendedWireInfo& ewi, const short& x, const short& y);
		void stitchInode(const Tilewire& twAnchor, const int &inodeXilinx, 
				t_rr_node &rrnXilinx, gzFile fpEdgeSwitch, std::vector<short>& switches);

	public:
		Xdlrc2Vpr(DDB& inDDB, const std::string &packageName, const std::string &postfix, const int width, const int height) : 
			mDDB(inDDB), mTiles(mDDB.getTiles()), mSegments(mDDB.getSegments()), 
			mPackageName(packageName), mPostfix(postfix), 
			mWidth(width), mHeight(height)
		{
			const string &dev = mDDB.getDeviceName();
			loadVpr(dev+packageName+postfix+".vpr");
			loadPtc2Name(dev+".ptc2name");

			if (boost::starts_with(mDDB.getDeviceName(), "xc6")) {
				re_BYP_B = boost::regex("BYP_B\\d");
			}
			else throw;
		}
		void operator()(void);

	};

} // namespace architecture
} // namespace torc

#endif // TORC_ARCHITECTURE_VPREXPORTER_HPP
