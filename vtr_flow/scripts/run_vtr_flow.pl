#!/usr/bin/perl
###################################################################################
# This script runs the VTR flow for a single benchmark circuit and architecture
# file.
#
# Usage:
#	run_vtr_flow.pl <circuit_file> <architecture_file> [OPTIONS]
#
# Parameters:
# 	circuit_file: Path to the input circuit file (verilog, blif, etc)
#   architecture_file: Path to the architecture file (.xml)
#
# Options:
# 	-starting_stage <stage>: Start the VTR flow at the specified stage.
#								Acceptable values: odin, abc, script, vpr, overlay, match, collapse, bitstream.
#								Default value is odin.
#   -ending_stage <stage>: End the VTR flow at the specified stage. Acceptable
#								values: odin, abc, script, vpr, overlay, match, collapse, bitstream.
#                               Default value is vpr.
# 	-keep_intermediate_files: Do not delete the intermediate files.
#
#   -temp_dir <dir>: Directory used for all temporary files
###################################################################################

############## KEY CHANGES BY EH FOR OVERLAY NETWORK ##############################                                         
# 1. In common with VTR upstream, technology-mapping at the ABC stage no longer performs area-recovery
#    (result is that circuits have significantly better delay).
# 2. Three final stages, "overlay", "match" and "collapse" have been added.
#    VPR now defaults to: "--allow_unrelated_clustering off".
# 3. The script has been changed to echo each stage's results onto screen as well as to file
#    (as before), and also uses "<arch_file>/<benchmark_name>" as the default temp_dir
###################################################################################

use strict;
use Cwd;
use File::Spec;
use POSIX;
use File::Copy;
use List::Util 'shuffle';  #added Qt
use FindBin;
use File::Which;
use File::Basename;
use Config;

my $time = "/usr/bin/time";

use Carp;
$SIG{ __DIE__ } = sub { Carp::confess( @_ ) };

use lib "$FindBin::Bin/perl_libs/XML-TreePP-0.41/lib";
use XML::TreePP;

# check the parametes.  Note PERL does not consider the script itself a parameter.
my $number_arguments = @ARGV;
if ( $number_arguments < 2 ) {
	print(
		"usage: run_vtr_flow.pl <circuit_file> <architecture_file> [OPTIONS]\n"
	);
	exit(-1);
}

my $time = "/usr/bin/time";

# Get Absoluate Path of 'vtr_flow
Cwd::abs_path($0) =~ m/(.*\/vtr_flow)\//;
my $vtr_flow_path = $1;
# my $vtr_flow_path = "./vtr_flow";

sub stage_index;
sub file_ext_for_stage;
sub expand_user_path;
sub file_find_and_replace;
sub xml_find_LUT_Kvalue;
sub xml_find_mem_size;

my $temp_dir = "";

my $stage_idx_odin      = 1;
my $stage_idx_abc       = 2;
my $stage_idx_ace       = 3;
my $stage_idx_prevpr    = 4;
my $stage_idx_vpr       = 5;
my $stage_idx_overlay   = 6;
my $stage_idx_match 	= 7;
my $stage_idx_collapse 	= 8;
my $stage_idx_bitstream = 9;

my $circuit_file_path      = expand_user_path( shift(@ARGV) );
my $architecture_file_path = expand_user_path( shift(@ARGV) );
my $sdc_file_path = "\"\"";

my $token;
my $ext;
my $starting_stage          = stage_index("odin");
my $ending_stage            = stage_index("bitstream");
my $keep_intermediate_files = 1;
my $has_memory              = 1;
my $timing_driven           = "on";
my $min_chan_width          = 18; 
my $mem_size                = -1;
my $lut_size                = -1;
my $vpr_cluster_seed_type   = "";
my $tech_file               = "";
my $do_power                = 0;
my $check_equivalent		= "off";
my $gen_postsynthesis_netlist 	= "off";

my $vpr_unrelated_clustering = "off";
my $vpr_options              = "";

# Channel width inflation over minimum
my $chan_width_inflation = 1.2;
# Fraction of total trace-buffer capacity to select signals for
my $trace_fraction = 0.75;
# Width of trigger
my $trigger_width = 32;
# Width of each memory block
my $memory_width = 72;
# Seed used to randomly select signals, if "time" is specified, use the current time as the seed
my $trace_seed = 0;
# Network connectivity (the max number of different trace-buffer inputs each net should be connected to)
my $overlay_connectivity = 20;
# Maximum number of routing iterations
my $overlay_iterations = 15;

my $seed			= 1;
my $min_hard_adder_size		= 4;
my @vpr_options             = qw(--allow_unrelated_clustering off);
my $vpr_fix_pins            = "random";
my $yosys_script            = "";
my $yosys_script_default    = "yosys.ys";
my $yosys_models            = "";
my $yosys_models_default    = "yosys_models.v";
my $yosys_abc_script        = "";
my $yosys_abc_script_default = "abc_vtr.rc";
my $abc_lut_file            = "abc_lut6.lut";

sub dbg {
    my ($caller, $filename, $line, $sub) = caller(1);
    $line = (caller 0)[2];
    my (@arg) = @_;
    if (!defined $arg[0]) {
        $arg[0] = "<undefined>";
    }
    my $msg;
    if (defined $sub && defined $line) {
        $msg = "[$sub:$line]+ @arg";
    } else {
        my ($caller, $filename, $line) = caller(0);
        $msg = "[$caller:$line]+ @arg";
    }
    $msg .= "\n" unless ($msg =~ /\n$/);
    print "$msg";

 }
 
while ( $token = shift(@ARGV) ) {
	if ( $token eq "-sdc_file" ) {
		$sdc_file_path = expand_user_path( shift(@ARGV) );
	}
	elsif ( $token eq "-starting_stage" ) {
		$starting_stage = stage_index( shift(@ARGV) );
	}
	elsif ( $token eq "-ending_stage" ) {
		$ending_stage = stage_index( shift(@ARGV) );
	}
	elsif ( $token eq "-keep_intermediate_files" ) {
		$keep_intermediate_files = 1;
	}
	elsif ( $token eq "-no_mem" ) {
		$has_memory = 0;
	}
	elsif ( $token eq "-no_timing" ) {
		$timing_driven = "off";
	}
	elsif ( $token eq "-vpr_route_chan_width" ) {
		$min_chan_width = shift(@ARGV);
	}
	elsif ( $token eq "-lut_size" ) {
		$lut_size = shift(@ARGV);
	}
	elsif ( $token eq "-vpr_cluster_seed_type" ) {
		$vpr_cluster_seed_type = shift(@ARGV);
	}
	elsif ( $token eq "-temp_dir" ) {
		$temp_dir = shift(@ARGV);
		print "temp_dir = $temp_dir\n"
	}
	elsif ( $token eq "-cmos_tech" ) {
		$tech_file = shift(@ARGV);
	}
	elsif ( $token eq "-power" ) {
		$do_power = 1;
	}
	elsif ( $token eq "-check_equivalent" ) {
		$check_equivalent = "on";
	}
	elsif ( $token eq "-gen_postsynthesis_netlist" ) {
		$gen_postsynthesis_netlist = "on";
	}
	elsif ( $token eq "-seed" ) {
		$seed = shift(@ARGV);
	}
	elsif ( $token eq "-min_hard_adder_size" ) {
		$min_hard_adder_size = shift(@ARGV);
	}
	elsif($token eq "-mem_size") {
		$mem_size = shift(@ARGV);
	}
	elsif($token eq "-vpr_unrelated_clustering"){
		$vpr_unrelated_clustering = shift(@ARGV);
	}
	elsif($token eq "-fast"){
		print "*******************************\n";
		print "***** FAST PLACEMENT MODE *****\n";                          
		print "*******************************\n";
		$vpr_options .= " --fast";
	}
	elsif ($token eq "-trace_fraction") 
	{
		$trace_fraction = shift(@ARGV);
	}
	elsif ($token eq "-trace_seed")
	{
		$trace_seed = shift(@ARGV);
	}
	elsif ($token eq "-overlay_connectivity")
	{
		$overlay_connectivity = shift(@ARGV);
	}
	elsif ($token eq "-overlay_iterations")
	{
		$overlay_iterations = shift(@ARGV);
	}
	elsif ($token eq "-trigger_width")
	{
		$trigger_width = shift(@ARGV);
	}                                         
	elsif ( $token eq "-yosys" ) {
		$yosys_script = $yosys_script_default;
	}
	elsif ( $token eq "-yosys_script" ) {
		$yosys_script = shift(@ARGV);
	}
	elsif ( $token eq "-yosys_models" ) {
		$yosys_models = shift(@ARGV);
	}
	elsif ( $token eq "-yosys_abc_script" ) {
		$yosys_abc_script = shift(@ARGV);
	}
	elsif ( $token eq "-abc_lut" ) {
		$abc_lut_file = shift(@ARGV);
	}
	elsif ( $token eq "-vpr_options" ) {
		push(@vpr_options, split(' ', shift(@ARGV)));
	}
	elsif ($token eq "-vpr_fix_pins") {
		$vpr_fix_pins = shift(@ARGV);
	}
	else {
		die "Error: Invalid argument ($token)\n";
	}

	if ( $starting_stage == -1 or $ending_stage == -1 ) {
		die
		  "Error: Invalid starting/ending stage name (start $starting_stage end $ending_stage).\n";
	}
}

if ( $ending_stage < $starting_stage ) {
	die "Error: Ending stage is before starting stage.";
}
if ($do_power) {
	if ( $tech_file eq "" ) {
		die "A CMOS technology behavior file must be provided.";
	}
	elsif ( not -r $tech_file ) {
		die "The CMOS technology behavior file ($tech_file) cannot be opened.";
	}
	$tech_file = Cwd::abs_path($tech_file);
}

if ( $vpr_cluster_seed_type eq "" ) {
	if ( $timing_driven eq "off" ) {
		$vpr_cluster_seed_type = "max_inputs";
	}
	else {
		$vpr_cluster_seed_type = "timing";
	}
}

# Get circuit name (everything up to the first '.' in the circuit file)  # added QT
my ( $vol, $path, $circuit_file_name ) =
  File::Spec->splitpath($circuit_file_path);
$circuit_file_name =~ m/(.*)[.].*?/;
my $benchmark_name = $1;

# Get architecture name
$architecture_file_path =~ m/.*\/(.*?.xml)/;
my $architecture_file_name = $1;

$architecture_file_name =~ m/(.*).xml$/;
my $architecture_name = $1;
print "$architecture_name/$benchmark_name...\n";

if ($temp_dir eq "")
{
	$temp_dir = "./$architecture_name/$benchmark_name";
}
if (! -d $temp_dir)
{
	system "mkdir -p $temp_dir";       
}

# Test for file existence
( -f $circuit_file_path )
  or die "Circuit file not found ($circuit_file_path)";
( -f $architecture_file_path )
  or die "Architecture file not found ($architecture_file_path)";

if ( $temp_dir eq "" ) {
	$temp_dir = basename($architecture_file_path,".xml")."/".basename($circuit_file_path,".v");
}
if ( !-d $temp_dir ) {
	system "mkdir -p $temp_dir";
}
-d $temp_dir or die "Could not make temporary directory ($temp_dir)\n";
if ( !( $temp_dir =~ /.*\/$/ ) ) {
	$temp_dir = $temp_dir . "/";
}

my $timeout      = 10 * 24 * 60 * 60;         # 10 day execution timeout
my $results_path = "${temp_dir}output.txt";

my $error;
my $error_code = 0;

my $arch_param;
my $cluster_size;
my $inputs_per_cluster = -1;


if ( !-e $sdc_file_path ) {
	# open( OUTPUT_FILE, ">$sdc_file_path" ); 
	# close ( OUTPUT_FILE );
	my $sdc_file_path;
}

my $vpr_path;
if ( $stage_idx_vpr >= $starting_stage and $stage_idx_vpr <= $ending_stage ) {
	$vpr_path = "$vtr_flow_path/../vpr/vpr";
	( -r $vpr_path or -r "${vpr_path}.exe" )
	  or die "Cannot find vpr exectuable ($vpr_path)";

  	if ($vpr_fix_pins ne "random") {
		( -r $vpr_fix_pins ) or die "Cannot find $vpr_fix_pins!";
		copy($vpr_fix_pins, $temp_dir);
		$vpr_fix_pins = basename($vpr_fix_pins);
	}
}

my $odin2_path;
my $odin_config_file_name;
my $odin_config_file_path;
my $yosys_path;
my $yosys_config_file_name;
my $yosys_config_file_path;
my $yosys_abc_script_file_path;

my $models_file_path_default;
my $models_file_path;
my $abc_rc_path;
my $yosys_abc_script_path;

if (    $stage_idx_odin >= $starting_stage
	and $stage_idx_odin <= $ending_stage )
{
	if ($yosys_script eq "") {
		$odin2_path = "$vtr_flow_path/../ODIN_II/odin_II.exe";
		( -e $odin2_path )
			or die "Cannot find ODIN_II executable ($odin2_path)";

		$odin_config_file_name = "basic_odin_config_split.xml";

		$odin_config_file_path = "$vtr_flow_path/misc/$odin_config_file_name";
		( -e $odin_config_file_path )
			or die "Cannot find ODIN config template ($odin_config_file_path)";

		$odin_config_file_name = "odin_config.xml";
		my $odin_config_file_path_new = "$temp_dir" . "odin_config.xml";
		copy( $odin_config_file_path, $odin_config_file_path_new );
		$odin_config_file_path = $odin_config_file_path_new;
	}
	else
	{
		$yosys_path = "$vtr_flow_path/../yosys/yosys";
		( -e $yosys_path )
			or die "Cannot find Yosys executable ($yosys_path)";

		$yosys_config_file_name = $yosys_script;
		$yosys_config_file_path = "$vtr_flow_path/misc/$yosys_config_file_name";
		( -e $yosys_config_file_path )
			or die "Cannot find Yosys script ($yosys_config_file_path)";

		my $yosys_config_file_path_new = "$temp_dir" . "$yosys_config_file_name";
		copy( $yosys_config_file_path, $yosys_config_file_path_new );
		$yosys_config_file_path = $yosys_config_file_path_new;

		my $tech_file_name;
		$tech_file_name = "single_port_ram.v";
		copy( "$vtr_flow_path/misc/$tech_file_name", "$temp_dir"."$tech_file_name" );
		$tech_file_name = "dual_port_ram.v";
		copy( "$vtr_flow_path/misc/$tech_file_name", "$temp_dir"."$tech_file_name" );
		$tech_file_name = "adder.v";
		copy( "$vtr_flow_path/misc/$tech_file_name", "$temp_dir"."$tech_file_name" );
		$tech_file_name = "multiply.v";
		copy( "$vtr_flow_path/misc/$tech_file_name", "$temp_dir"."$tech_file_name" );
		$tech_file_name = "lut7.v";
		copy( "$vtr_flow_path/misc/$tech_file_name", "$temp_dir"."$tech_file_name" );
		$tech_file_name = "lut8.v";
		copy( "$vtr_flow_path/misc/$tech_file_name", "$temp_dir"."$tech_file_name" );
		$tech_file_name = "xadder.v";
		copy( "$vtr_flow_path/misc/$tech_file_name", "$temp_dir"."$tech_file_name" );
		$tech_file_name = "bufgctrl.v";
		copy( "$vtr_flow_path/misc/$tech_file_name", "$temp_dir"."$tech_file_name" );
		$tech_file_name = "adder2xadder.v";
		copy( "$vtr_flow_path/misc/$tech_file_name", "$temp_dir"."$tech_file_name" );

		my $models_file_name = $yosys_models_default;
		$models_file_path_default = "$temp_dir"."$models_file_name";
		copy( "$vtr_flow_path/misc/$models_file_name", "$models_file_path_default" );

		$models_file_name = $yosys_models;
		if ($models_file_name ne "") {
			$models_file_path = "$temp_dir"."$models_file_name";
			copy( "$vtr_flow_path/misc/$models_file_name", "$models_file_path" );
		}

		if ($yosys_abc_script eq "") { 
			$yosys_abc_script = $yosys_abc_script_default;
		}
		$yosys_abc_script_path = "$temp_dir"."$yosys_abc_script";
		copy( "$vtr_flow_path/misc/$yosys_abc_script", $yosys_abc_script_path );
	}
}

######added Qt
my $mwbm_path = "$vtr_flow_path/../mwbm/mwbm";
(-e $mwbm_path) or die "Cannot find mwbm exectuable ($mwbm_path)";
######

my $abc_path;
$abc_rc_path = "$vtr_flow_path/../abc_with_bb_support/abc.rc";
( -e $abc_rc_path ) or die "Cannot find ABC RC file ($abc_rc_path)";
copy( $abc_rc_path, $temp_dir );

my $abc_lut_path = "$vtr_flow_path/misc/$abc_lut_file";
( -e $abc_lut_path ) or die "Cannot find ABC LUT file ($abc_lut_path)";
copy( $abc_lut_path, $temp_dir );

$abc_path = "$vtr_flow_path/../abc_with_bb_support/abc";
if ( $stage_idx_abc >= $starting_stage and $stage_idx_abc <= $ending_stage ) {
	( -e $abc_path or -e "${abc_path}.exe" )
	  or die "Cannot find ABC executable ($abc_path)";
}

my $ace_path;
if ( $stage_idx_ace >= $starting_stage and $stage_idx_ace <= $ending_stage and $do_power) {
	$ace_path = "$vtr_flow_path/../ace2/ace";
	( -e $ace_path or -e "${ace_path}.exe" )
	  or die "Cannot find ACE executable ($ace_path)";
}

my $bitstream_path = "$vtr_flow_path/../bnpr2xdl/bnpr2xdl";
my ($xdl_path, $par_path, $trce_path, $bitgen_path);
my $arch = basename($architecture_file_path,".xml");
if ( $stage_idx_bitstream >= $starting_stage and $stage_idx_bitstream <= $ending_stage ) {
	(-e $bitstream_path) or die "Warning: Cannot find $bitstream_path. Please run \"make\" again.";

	$xdl_path = which("xdl") or die "Cannot find xdl exectuable on \$PATH\n";

	$trce_path = which("trce") or die "Cannot find trce exectuable on \$PATH\n";

	$bitgen_path = which("bitgen") or die "Cannot find bitgen exectuable on \$PATH\n";
}


# Get Memory Size
my $mem_size = -1;
my $line;
my $in_memory_block;
my $in_mode;

# Read arch XML
my $tpp      = XML::TreePP->new();
my $xml_tree = $tpp->parsefile($architecture_file_path);

# Get lut size
if ( $lut_size < 1 ) {
	$lut_size = xml_find_LUT_Kvalue($xml_tree);
	if ( $lut_size < 1 ) {
		print "failed: cannot determine arch LUT k-value";
		$error_code = 1;
	}
}
print "LUT size: $lut_size\n";

# Get memory size
$mem_size = xml_find_mem_size($xml_tree);
print "MEM size: $mem_size\n";
print "Min Hard Adder size: $min_hard_adder_size\n";

my $odin_output_file_name =
  "$benchmark_name" . file_ext_for_stage($stage_idx_odin);
my $odin_output_file_path = "$temp_dir$odin_output_file_name";

my $abc_output_file_name =
  "$benchmark_name" . file_ext_for_stage($stage_idx_abc);
my $abc_output_file_path = "$temp_dir$abc_output_file_name";

my $ace_output_blif_name =
  "$benchmark_name" . file_ext_for_stage($stage_idx_ace);
my $ace_output_blif_path = "$temp_dir$ace_output_blif_name";

my $ace_output_act_name = "$benchmark_name" . ".act";
my $ace_output_act_path = "$temp_dir$ace_output_act_name";

my $prevpr_output_file_name =
  "$benchmark_name" . file_ext_for_stage($stage_idx_prevpr);
my $prevpr_output_file_path = "$temp_dir$prevpr_output_file_name";

my $vpr_route_output_file_name = "$benchmark_name.route";
my $vpr_route_output_file_path = "$temp_dir$vpr_route_output_file_name";

#my $vpr_place_output_file_path = "$temp_dir$benchmark_name.place";

#system "cp $abc_rc_path $temp_dir";
#system "cp $architecture_path $temp_dir";
#system "cp $circuit_path $temp_dir/$benchmark_name" . file_ext_for_stage($starting_stage - 1);
#system "cp $odin2_base_config"

my $architecture_file_path_new = "$temp_dir$architecture_file_name";
copy( $architecture_file_path, $architecture_file_path_new );
my $architecture_file_path_orig = $architecture_file_path;
$architecture_file_path = $architecture_file_path_new;

my $circuit_file_path_new =
  "$temp_dir$benchmark_name" . file_ext_for_stage(0);
copy( $circuit_file_path, $circuit_file_path_new );
$circuit_file_path = $circuit_file_path_new;

if ($vpr_fix_pins ne "random")
{
		copy($vpr_fix_pins, $temp_dir);
}

# Call executable and time it
my $StartTime = time;
my $q         = "not_run";

#################################################################################
################################## ODIN #########################################
#################################################################################

if ( $starting_stage <= $stage_idx_odin and !$error_code ) {

	unlink "$odin_output_file_path";
	if ($yosys_script eq "") {
		#system "sed 's/XXX/$benchmark_name.v/g' < $odin2_base_config > temp1.xml";
		#system "sed 's/YYY/$arch_name/g' < temp1.xml > temp2.xml";
		#system "sed 's/ZZZ/$odin_output_file_path/g' < temp2.xml > temp3.xml";
		#system "sed 's/PPP/$mem_size/g' < temp3.xml > circuit_config.xml";

		file_find_and_replace( $odin_config_file_path, "XXX", $circuit_file_name );
		file_find_and_replace( $odin_config_file_path, "YYY",
			$architecture_file_name );
		file_find_and_replace( $odin_config_file_path, "ZZZ",
			$odin_output_file_name );
		file_find_and_replace( $odin_config_file_path, "PPP", $mem_size );
		file_find_and_replace( $odin_config_file_path, "AAA", $min_hard_adder_size );

		if ( !$error_code ) {
			$q =
			&system_with_timeout( "$odin2_path", "odin.out", $timeout, $temp_dir,
				"-c", $odin_config_file_name );

			if ( -e $odin_output_file_path ) {
				dbg "chkpoint 1\n";
				if ( !$keep_intermediate_files ) {
					system "rm -f ${temp_dir}*.dot";
					system "rm -f ${temp_dir}*.v";
					system "rm -f $odin_config_file_path";
					dbg "chkpoint 2\n";
				}
			}
			else {
				print "failed: odin";
				$error_code = 1;
			}
			print "succeeded: odin\n";
		}
	}
	else {
		file_find_and_replace( $yosys_config_file_path, "XXX", $circuit_file_name );
		file_find_and_replace( $yosys_config_file_path, "ZZZ",
			$odin_output_file_name );
		#file_find_and_replace( $yosys_config_file_path, "LUTSIZE", $lut_size );
		file_find_and_replace( $yosys_config_file_path, "ABCEXE", $abc_path );
		file_find_and_replace( $yosys_config_file_path, "ABCSCRIPT", $yosys_abc_script );

		file_find_and_replace( $yosys_abc_script_path, "ABCLUT", $abc_lut_file );

		file_find_and_replace( $models_file_path_default, "PPP", $mem_size );
		file_find_and_replace( $models_file_path_default, "AAA", $min_hard_adder_size );
		if ($models_file_path ne "") {
			file_find_and_replace( $models_file_path, "PPP", $mem_size );
			file_find_and_replace( $models_file_path, "AAA", $min_hard_adder_size );
		}

		if ( !$error_code ) {
			$q =
			&system_with_timeout( "$yosys_path", "yosys.out", $timeout, $temp_dir,
				"-v 2", $yosys_config_file_name );

			if ( -e $odin_output_file_path ) {
				if ( !$keep_intermediate_files ) {
					system "rm -f ${temp_dir}*.dot";
					system "rm -f ${temp_dir}*.v";
					system "rm -f $odin_config_file_path";
				}
				print "Succeeded: Yosys\n";
			}
			else {
				print "failed: yosys";
				$error_code = 1;
			}
		}

	}
}

#################################################################################
################################## ABC ##########################################
#################################################################################
if (    $starting_stage <= $stage_idx_abc
	and $ending_stage >= $stage_idx_abc
	and !$error_code )
{
	print "chkpoint 3\n";
	if ($yosys_script eq "") {
		# EH: Replace all .subckt adder with .subckt xadder, 
		# with XOR and AND pushed into soft-logic
		my $abc_input_file_name = "$benchmark_name" . ".xadder" . file_ext_for_stage($stage_idx_odin);
		my $abc_input_file_path = "$temp_dir$abc_input_file_name";

		my $adder_model = <<'EOF';
(\.model adder
\.inputs\s+a(\[0\])?\s+b(\[0\])?\s+cin(\[0\])?
\.outputs\s+cout(\[0\])?\s+sumout(\[0\])?)
\.blackbox
\.end
EOF
		my $xadder_model = <<'EOF';
\1
.names a\2 b\3 a_xor_b
01 1
10 1
.names a\2 b\3 a_and_b
11 1
.subckt xadder a_xor_b=a_xor_b a_and_b=a_and_b cin=cin\4 cout=cout\5 sumout=sumout\6
.end

.model xadder
.inputs a_xor_b a_and_b cin
.outputs cout sumout
.blackbox
.end
EOF
		
		unlink "$abc_input_file_path";
		copy( $odin_output_file_path, $abc_input_file_path );
		
		&system_with_timeout("/usr/bin/perl", "perl.out", $timeout, $temp_dir, 
			"-0777", "-p", "-i", "-e", "\"s/$adder_model/$xadder_model/smg\"", $abc_input_file_name);

		print "chkpoint 5\n";
		unlink "$abc_output_file_path";
		
		$q = &system_with_timeout( $abc_path, "abc.out", $timeout, $temp_dir, "-c",
			"\"read $abc_input_file_name; read_lut $abc_lut_file; time; resyn; resyn2; if -K $lut_size; time; scleanup; time; scleanup; time; scleanup; time; scleanup; time; scleanup; time; scleanup; time; scleanup; time; scleanup; time; scleanup; time; print_stats; write_hie $abc_input_file_name $abc_output_file_name\""
		);
		dbg "chkpoint 6 \n"; 
	}
	else
	{
		unlink "$abc_output_file_path";
		$q = &system_with_timeout( $abc_path, "abc.out", $timeout, $temp_dir, "-c",
			"read $odin_output_file_name; print_stats; write_hie $odin_output_file_name $abc_output_file_name"
		);
	}

	if ( -e $abc_output_file_path ) {
		print "chkpoint 7 \n";
		#system "rm -f abc.out";
		if ( !$keep_intermediate_files ) {
			system "rm -f $odin_output_file_path";
			system "rm -f ${temp_dir}*.rc";
			print "chkpoint 8 \n";
		}
		print "succeeded: abc\n";
	}
	else {
		print "failed: abc";
		$error_code = 1;
	}
}

#################################################################################
################################## ACE ##########################################
#################################################################################
if (    $starting_stage <= $stage_idx_ace
	and $ending_stage >= $stage_idx_ace
	and $do_power
	and !$error_code )
{
	$q = &system_with_timeout(
		$ace_path, "ace.out",             $timeout, $temp_dir,
		"-b",      $abc_output_file_name, "-n",     $ace_output_blif_name,
		"-o",      $ace_output_act_name
	);

	if ( -e $ace_output_blif_path ) {
		if ( !$keep_intermediate_files ) {
			system "rm -f $abc_output_file_path";
			#system "rm -f ${temp_dir}*.rc";
		}
		print "succeeded: scripts\n";
	}
	else {
		print "failed: ace";
		$error_code = 1;
	}
}

#################################################################################
################################## PRE-VPR ######################################
#################################################################################
if (    $starting_stage <= $stage_idx_prevpr
	and $ending_stage >= $stage_idx_prevpr
	and !$error_code )
{
	my $prevpr_success   = 1;
	my $prevpr_input_blif_path;
	if ($do_power) {
		$prevpr_input_blif_path = $ace_output_blif_path; 
	} else {
		$prevpr_input_blif_path = $abc_output_file_path;
	}
	
	if ($yosys_script eq "") {
		# EH: Scan all .latch -es for clocks
		# Add a BUFGCTRL for each clock found
		open (my $fin, $prevpr_input_blif_path) or die ("Could not open $prevpr_input_blif_path");
		open (my $fout, ">$prevpr_output_file_path") or die ("Could not open $prevpr_output_file_path");
		my %clks;
		while (my $line = <$fin>) {
			chomp $line;
			$line =~ m/(\s*)\.latch(\s+)([^ ]+)(\s+)([^ ]+)(\s+)([^ ]+)(\s+)([^ ]+)(\s+)([^ ]+)$/;
			if ($9) {
				$clks{$9} = 1;
			}
			foreach my $clk (keys %clks) {
				$line =~ s/\Q$clk /${clk}_BUFG /g;
			}
			print $fout "$line\n";
		}
		close $fin;
		if (keys %clks) {
			print $fout "\n";
			foreach my $clk (keys %clks) {
				print $fout ".subckt bufgctrl i[0]=$clk i[1]=unconn s[0]=unconn s[1]=unconn ce[0]=unconn ce[1]=unconn ignore[0]=unconn ignore[1]=unconn o[0]=$clk"."_BUFG\n";
			}
			print $fout "\n";
			print $fout ".model bufgctrl\n";
			print $fout ".inputs i[0] i[1] s[0] s[1] ce[0] ce[1] ignore[0] ignore[1]\n";
			print $fout ".outputs o[0]\n";
			print $fout ".blackbox\n";
			print $fout ".end\n";
		}
		close $fout;
	}
	else {
		copy($prevpr_input_blif_path, $prevpr_output_file_path);
	}

	if ($prevpr_success) {
		if ( !$keep_intermediate_files ) {
			system "rm -f $prevpr_input_blif_path";
		}
		print ("succeeded: pre-vpr\n");
	}
	else {
		print "failed: prevpr";
		$error_code = 1;
	}
}

#################################################################################
################################## VPR ##########################################
#################################################################################

if ( $starting_stage <= $stage_idx_vpr 
	and $ending_stage >= $stage_idx_vpr 
	and !$error_code ) 
{
	(my $rrg_file_path = File::Spec->rel2abs($architecture_file_path_orig)) =~ s{\.[^.]+$}{.rrg.gz};
	(-e "$rrg_file_path") or die("$rrg_file_path does not exist!");
	unless(-e "$temp_dir/".basename($rrg_file_path)) {
		symlink($rrg_file_path, "$temp_dir/".basename($rrg_file_path)) or die;
	}

	my @vpr_power_args;

	if ($do_power) {
		push( @vpr_power_args, "--power" );
		push( @vpr_power_args, "--tech_properties" );
		push( @vpr_power_args, "$tech_file" );
	}
	if ( $min_chan_width < 0 ) {
		
		#if default BLIF file does not exist, try again **** For Added Qt/removed unrelated_clustering option below in system sub call
		#if (! -e "$temp_dir$scripts_output_file_name") 
		#{
		#	$scripts_output_file_name = "$benchmark_name" . file_ext_for_stage($stage_idx_script);
		#	(-e "$temp_dir$scripts_output_file_name") or die "Cannot find file $temp_dir$scripts_output_file_name";
		#}
		
		$q = &system_with_timeout(
			$vpr_path,                    "vpr.out",
			$timeout,                     $temp_dir,
			$architecture_file_name,      "$benchmark_name",
			"--blif_file",				  "$prevpr_output_file_name",
			"--timing_analysis",          "$timing_driven",
			"--timing_driven_clustering", "$timing_driven",
			"--cluster_seed_type",        "$vpr_cluster_seed_type",
			"--sdc_file", 				  "$sdc_file_path",
			"--seed",			 		  "$seed",
			@vpr_options,
			"--fix_pins", $vpr_fix_pins,
			"--inc_dump_all_nets",
			"--nodisp"
		);
		dbg "min_chan_width: $min_chan_width \n"; 
		dbg "chkpoint 7 \n";
		
		if ( $timing_driven eq "on" ) {
			# Critical path delay is nonsensical at minimum channel width because congestion constraints completely dominate the cost function.
			# Additional channel width needs to be added so that there is a reasonable trade-off between delay and area
			# Commercial FPGAs are also desiged to have more channels than minimum for this reason

			# Parse out min_chan_width
			dbg "chkpoint 8 \n";
			if ( open( VPROUT, "<${temp_dir}vpr.out" ) ) {
				undef $/;
				my $content = <VPROUT>;
				close(VPROUT);
				$/ = "\n";    # Restore for normal behaviour later in script

				if ( $content =~ m/(.*Error.*)/i ) {
					$error = $1;
				}

				if ( $content =~
					/Best routing used a channel width factor of (\d+)/m )
				{
					$min_chan_width = $1;
				}
			}

			$min_chan_width = ( $min_chan_width * $chan_width_inflation );
			$min_chan_width = floor($min_chan_width);
			if ( $min_chan_width % 2 ) {
				$min_chan_width = $min_chan_width + 1;
			}
			
			dbg "min_chan_width after parse: $min_chan_width \n"; 
			dbg "chkpoint 9 \n";
			
			if ( -e $vpr_route_output_file_path ) {
				dbg "chkpoint 10 \n";
				system "rm -f $vpr_route_output_file_path";
				$q = &system_with_timeout(
					$vpr_path,               "vpr.crit_path.out",
					$timeout,                $temp_dir,
					$architecture_file_name, "$benchmark_name",
					"--route",
					"--blif_file",           "$prevpr_output_file_name",
					"--route_chan_width",    "$min_chan_width",
					#"--cluster_seed_type",   "$vpr_cluster_seed_type",
					"--max_router_iterations", "100",
					"--nodisp",              @vpr_power_args,
					"--gen_postsynthesis_netlist", "$gen_postsynthesis_netlist",
					"--sdc_file",			 "$sdc_file_path"
				);
			}
		}
	}
	else {
		$q = &system_with_timeout(
			$vpr_path,                    "vpr.out",
			$timeout,                     $temp_dir,
			$architecture_file_name,      "$benchmark_name",
			"--blif_file",                "$prevpr_output_file_name",
			"--timing_analysis",          "$timing_driven",
			"--timing_driven_clustering", "$timing_driven",
			"--inc_dump_all_nets",
			"--route_chan_width",         "$min_chan_width",
			"--nodisp",                   "--cluster_seed_type",
			"$vpr_cluster_seed_type",     @vpr_power_args,
			"--gen_postsynthesis_netlist", "$gen_postsynthesis_netlist",
			"--sdc_file",				  "$sdc_file_path",
			"--seed",			"$seed",
			"--fix_pins",			"$vpr_fix_pins",
			@vpr_options
		);
	}
	
	if (-e $vpr_route_output_file_path and
		$q eq "success")                       #possibly a bug - now understood 	
	{
		dbg " route file by vpr: $vpr_route_output_file_path \n";
		#dbg " place file by vpr: $vpr_place_output_file_path \n";
		
		if($check_equivalent eq "on") {
			if($abc_path eq "") {
				$abc_path = "$vtr_flow_path/../abc_with_bb_support/abc";
			}
			$q = &system_with_timeout($abc_path, 
							"equiv.out",
							$timeout,
							$temp_dir,
							"-c", 
							"cec $prevpr_output_file_name post_pack_netlist.blif;sec $prevpr_output_file_name post_pack_netlist.blif"
			);
		}
		if (! $keep_intermediate_files)
		{
			system "rm -f $prevpr_output_file_name";
			system "rm -f ${temp_dir}*.xml";
			system "rm -f ${temp_dir}*.net";
			system "rm -f ${temp_dir}*.place";
			system "rm -f ${temp_dir}*.route";
			system "rm -f ${temp_dir}*.sdf";
			system "rm -f ${temp_dir}*.v";
			if ($do_power) {
				system "rm -f $ace_output_act_path";
			}
		}
		print ("succeeded: vpr\n");
	}
	else {
		print("failed: vpr");
		$error_code = 1;
	}
}

my $EndTime = time;

# Determine running time
my $seconds    = ( $EndTime - $StartTime );
my $runseconds = $seconds % 60;

# Start collecting results to output.txt
open( RESULTS, "> $results_path" );

# Output vpr status and runtime
print RESULTS "vpr_status=$q\n";
print RESULTS "vpr_seconds=$seconds\n";

# Parse VPR output
if ( open( VPROUT, "< vpr.out" ) ) {
	undef $/;
	my $content = <VPROUT>;
	close(VPROUT);
	$/ = "\n";    # Restore for normal behaviour later in script

	if ( $content =~ m/(.*Error.*)/i ) {
		$error = $1;
	}
}
print RESULTS "error=$error\n";

close(RESULTS);

# Clean up files not used that take up a lot of space

#system "rm -f *.blif";
#system "rm -f *.xml";
#system "rm -f core.*";
#system "rm -f gc.txt";

##added QT here
 
#################################################################################
############################ INSERT OVERLAY NETWORK  ############################
#################################################################################

if ($starting_stage <= $stage_idx_overlay and $ending_stage >= $stage_idx_overlay and ! $error_code)
{
	# Read previous channel width
	#open(VPROUT, "<${temp_dir}vpr.crit_path.out") or die "Cannot open ${temp_dir}vpr.crit_path.out";
	#undef $/;
	#my $content = <VPROUT>;
	#close (VPROUT);
	#$/ = "\n";     # Restore for normal behaviour later in script
	#
	#if ($content =~ m/(.*Error.*)/i) {
	#	$error = $1;
	#}
	#
	#$content =~ /Circuit successfully routed with a channel width factor of (\d+)/m or die;
	#$min_chan_width = $1 ; 

	$q = &system_with_timeout($vpr_path, 
				"overlay.out",
				$timeout,
				$temp_dir,
				$architecture_file_name,      "$benchmark_name",
				"--blif_file",                "$prevpr_output_file_name",
				"--route",
				"--router_algorithm", "read_route",
				"--route_chan_width", $min_chan_width,
				"--timing_analysis", "$timing_driven",
				"-inc_instrument_type", "overlay",
				"-inc_connectivity", "$overlay_connectivity",
				"-max_router_iterations", "$overlay_iterations",
				"--nodisp"
	);
}

#################################################################################
############################# MATCH OVERLAY NETWORK #############################
#################################################################################

if ($starting_stage <= $stage_idx_match and $ending_stage >= $stage_idx_match and ! $error_code)
{
	print "\n";

	# Read in all vnet signals
	open(MAP, "<$temp_dir$benchmark_name".".map") or die "Cannot open $temp_dir$benchmark_name".".map";
	my @nets = ();
	while (<MAP>) {
		my $vnet = (split(',', $_))[0];
		push(@nets, $vnet);
	}
	close(MAP);
	my $total_nets = @nets;

	if ($trace_seed eq "time")
	{
		$trace_seed = time ^ $$;
	}

	$q = &system_with_timeout($mwbm_path, 
				"match.out",
				$timeout,
				$temp_dir,
				"$benchmark_name.overlay",
				"$trigger_width",
				"$total_nets",
				"$trace_fraction",
				"$trace_seed"
	);
}

#################################################################################
############################ COLLAPSE OVERLAY NETWORK ###########################
#################################################################################

if ($ending_stage >= $stage_idx_collapse and ! $error_code)
{
	# Read previous channel width
	open(VPROUT, "<${temp_dir}vpr.crit_path.out") or die "Cannot open ${temp_dir}vpr.crit_path.out";
	undef $/;
	my $content = <VPROUT>;
	close (VPROUT);
	$/ = "\n";     # Restore for normal behaviour later in script
	
	if ($content =~ m/(.*Error.*)/i) {
		$error = $1;
	}
	
	$content =~ /Circuit successfully routed with a channel width factor of (\d+)/m or die;
	$min_chan_width = $1 ;

	$q = &system_with_timeout($vpr_path, 
				"collapse.out",
				$timeout,
				$temp_dir,
				$architecture_file_name,
				"$benchmark_name",
				"--route",
				"--router_algorithm", "read_route",
				"--route_chan_width", $min_chan_width,
				"--timing_analysis", "$timing_driven",
				"--inc_instrument_type", "overlay",
#				"-inc_connectivity", "$overlay_connectivity",
#				"-max_router_iterations", "$overlay_iterations",
				"--inc_router_algorithm" , "read_route",
				"--inc_route_file", "${benchmark_name}.overlay.route",
				"--inc_match_file", "${benchmark_name}.overlay.match",
				"--nodisp"
	);
}

#################################################################################
################################## BITSTREAM ####################################
#################################################################################

if ($ending_stage >= $stage_idx_bitstream and ! $error_code)
{
	(my $pkg_file_path = File::Spec->rel2abs($architecture_file_path_orig)) =~ s{(_[^_/]+)?\.[^.]+$}{.pkg};
	(-e "$pkg_file_path") or die("$pkg_file_path does not exist!");
	unless (-e "$temp_dir/".basename($pkg_file_path)) {
		symlink($pkg_file_path, "$temp_dir/".basename($pkg_file_path)) or die;
	}

	(my $tws_file_path = File::Spec->rel2abs($architecture_file_path_orig)) =~ s{\.[^.]+$}{.tws};
	(-e "$tws_file_path") or die("$tws_file_path does not exist!");
	unless(-e "$temp_dir/".basename($tws_file_path)) {
		symlink($tws_file_path, "$temp_dir/".basename($tws_file_path)) or die;
	}

	my @bitgen_options;
	if ($arch eq "xc7z020clg484") {
		unless (-e "$temp_dir/xc7z020.db") {
			symlink("$vtr_flow_path/arch/xilinx/xc7z020.db", "$temp_dir/xc7z020.db") or die;
		}
		unless (-e "$temp_dir/Zynq7000.db") {
			symlink("$vtr_flow_path/arch/xilinx/Zynq7000.db", "$temp_dir/Zynq7000.db") or die;
		}

		push (@bitgen_options, qw(-g UnconstrainedPins:Allow));
	}
	elsif ($arch eq "xc6vlx240tff1156") {
		unless (-e "$temp_dir/xc6vlx240t.db") {
			symlink("$vtr_flow_path/arch/xilinx/xc6vlx240t.db", "$temp_dir/xc6vlx240t.db") or die;
		}
		unless (-e "$temp_dir/Virtex6.db") {
			symlink("$vtr_flow_path/arch/xilinx/Virtex6.db", "$temp_dir/Virtex6.db") or die;
		}
		unless (-e "$temp_dir/xc6vlx240tff1156_include.xdl") {
			symlink("$vtr_flow_path/arch/xilinx/xc6vlx240tff1156_include.xdl", "$temp_dir/xc6vlx240tff1156_include.xdl") or die;
		}
	}
	else {
		die($arch);
	}

	(-e "$prevpr_output_file_path") or die("$prevpr_output_file_path does not exist!");
	(-e "$temp_dir$benchmark_name.net") or die("$temp_dir$benchmark_name.net does not exist!");
	(-e "$temp_dir$benchmark_name.place") or die("$temp_dir$benchmark_name.place does not exist!");
	(-e "$temp_dir$benchmark_name.route") or die("$temp_dir$benchmark_name.route does not exist!");

	unlink "$temp_dir$benchmark_name".".xdl"; 
	$q = &system_with_timeout($bitstream_path, 
					"bitstream.out",
					$timeout,
					$temp_dir,
					$arch,
					$benchmark_name
	);
	(-e "$temp_dir$benchmark_name".".xdl") or die("$temp_dir$benchmark_name".".xdl does not exist!");

	unlink "$temp_dir$benchmark_name".".ncd"; 
	$q = &system_with_timeout(	
			$xdl_path, 
			"xdl2ncd.out",
			$timeout,
			$temp_dir,
			"-force",
			"-xdl2ncd",
			"$benchmark_name".".xdl",
			"$benchmark_name".".ncd"
	);
	
	(-e "$temp_dir$benchmark_name".".ncd") or die("$temp_dir$benchmark_name".".ncd does not exist!");

	$q = &system_with_timeout(
			$trce_path, 
			"trce.out",
			$timeout,
			$temp_dir,
			"-v", "10",
			"-a",
			"$benchmark_name.ncd"
			);

	unlink "$temp_dir$benchmark_name.bit"; 
	unlink "$temp_dir$benchmark_name.drc"; 
	$q = &system_with_timeout(
			$bitgen_path, 
			"bitgen.out",
			$timeout,
			$temp_dir,
			"-d",
			@bitgen_options,
			"-w", "$benchmark_name.ncd",
			);

	(-e "$temp_dir$benchmark_name.bit") or die("$temp_dir$benchmark_name.ncd does not exist!");
}

if ( !$error_code ) {
	#system "rm -f *.echo";
	print "OK";
}
print "\n";

################################################################################
# Subroutine to execute a system call with a timeout
# system_with_timeout(<program>, <stdout file>, <timeout>, <dir>, <arg1>, <arg2>, etc)
#    make sure args is an array
# Returns: "timeout", "exited", "success", "crashed"
################################################################################
sub system_with_timeout {

	# Check args
	( $#_ > 2 )   or die "system_with_timeout: not enough args\n";
	#( -f $_[0] )  or die "system_with_timeout: can't find executable $_[0]\n";
	(-f $_[0]) or which($_[0]) or die "system_with_timeout: can't find executable\n";
	( $_[2] > 0 ) or die "system_with_timeout: invalid timeout\n";

	# Save the pid of child process
	my $pid = fork;

	if ( $pid == 0 ) {

		# Redirect STDOUT for vpr
		chdir $_[3];

		
		#open( STDOUT, "| tee $_[1]" );
		#open( STDERR, ">&STDOUT" );
		

		# Copy the args and cut out first four
		my @VPRARGS = @_;
		shift @VPRARGS;
		shift @VPRARGS;
		shift @VPRARGS;
		shift @VPRARGS;

		# Run command - added Qt
		# This must be an exec call and there most be no special shell characters
		# like redirects so that perl will use execvp and $pid will actually be
		# that of vpr so we can kill it later.
		
		#print "\n$_[0] @VPRARGS\n";
		#exec "/usr/bin/time", "-v", $_[0], @VPRARGS;
		
		open(FILE, ">$_[1]");
			# dbg "gert 1";
			print FILE "$time $_[0]  @VPRARGS 2>&1\n";
			# dbg " gert 2";
		close(FILE);
			# dbg "$time $_[0]  @VPRARGS 2>&1 | tee -a $_[1]";
			exec "$time $_[0]  @VPRARGS 2>&1 | tee -a $_[1]";
			# dbg "gert 3";
	}
	else {
		my $timed_out = "false";

		# Register signal handler, to kill child process (SIGABRT)
		$SIG{ALRM} = sub { kill 6, $pid; $timed_out = "true"; };

		# Register handlers to take down child if we are killed (SIGHUP)
		$SIG{INTR} = sub { print "SIGINTR\n"; kill 1, $pid; exit; };
		$SIG{HUP}  = sub { print "SIGHUP\n";  kill 1, $pid; exit; };

		# Set SIGALRM timeout
		alarm $_[2];

		# Wait for child process to end OR timeout to expire
		wait;

		# Unset the alarm in case we didn't timeout
		alarm 0;

		# Check if timed out or not
		if ( $timed_out eq "true" ) {
			return "timeout";
		}
		else {
			my $did_crash = "false";
			if ( $? & 127 ) { $did_crash = "true"; }

			my $return_code = $? >> 8;

			if ( $did_crash eq "true" ) {
				return "crashed";
			}
			elsif ( $return_code != 0 ) {
				return "exited";
			}
			else {
				return "success";
			}
		}
	}
}

sub stage_index {
	my $stage_name = $_[0];

	if ( lc($stage_name) eq "odin" ) {
		return $stage_idx_odin;
	}
	if ( lc($stage_name) eq "abc" ) {
		return $stage_idx_abc;
	}
	if ( lc($stage_name) eq "ace" ) {
		return $stage_idx_ace;
	}
	if ( lc($stage_name) eq "prevpr" ) {
		return $stage_idx_prevpr;
	}
	##### added Qt 
	if ( lc($stage_name) eq "vpr" ) {
		return $stage_idx_vpr;
	}
	if (lc($stage_name) eq "overlay")
	{
		return $stage_idx_overlay;
	}
	if (lc($stage_name) eq "match")
	{
		return $stage_idx_match;
	}
	if (lc($stage_name) eq "collapse")
	{
		return $stage_idx_collapse;
	}
	if ( lc($stage_name) eq "bitstream" ) {
		return $stage_idx_bitstream;
	}
	return -1;
}

sub file_ext_for_stage {
	my $stage_idx = $_[0];

	if ( $stage_idx == 0 ) {
		return ".v";
	}
	elsif ( $stage_idx == $stage_idx_odin ) {
		return ".odin.blif";
	}
	elsif ( $stage_idx == $stage_idx_abc ) {
		return ".abc.blif";
	}
	elsif ( $stage_idx == $stage_idx_ace ) {
		return ".ace.blif";
	}
	elsif ( $stage_idx == $stage_idx_prevpr ) {
		return ".pre-vpr.blif";
	}
}

sub expand_user_path {
	my $str = shift;
	$str =~ s/^~\//$ENV{"HOME"}\//;
	return $str;
}

sub file_find_and_replace {
	my $file_path      = shift();
	my $search_string  = shift();
	my $replace_string = shift();

	open( FILE_IN, "$file_path" );
	my $file_contents = do { local $/; <FILE_IN> };
	close(FILE_IN);

	$file_contents =~ s/$search_string/$replace_string/mg;

	open( FILE_OUT, ">$file_path" );
	print FILE_OUT $file_contents;
	close(FILE_OUT);
}

sub xml_find_key {
	my $tree = shift();
	my $key  = shift();

	foreach my $subtree ( keys %{$tree} ) {
		if ( $subtree eq $key ) {
			return $tree->{$subtree};
		}
	}
	return "";
}

sub xml_find_child_by_key_value {
	my $tree = shift();
	my $key  = shift();
	my $val  = shift();

	if ( ref($tree) eq "HASH" ) {

		# Only a single item in the child array
		if ( $tree->{$key} eq $val ) {
			return $tree;
		}
	}
	elsif ( ref($tree) eq "ARRAY" ) {

		# Child Array
		foreach my $child (@$tree) {
			if ( $child->{$key} eq $val ) {
				return $child;
			}
		}
	}

	return "";
}

sub xml_find_LUT_Kvalue {
	my $tree = shift();

	#Check if this is a LUT
	if ( xml_find_key( $tree, "-blif_model" ) eq ".names" ) {
		return $tree->{input}->{"-num_pins"};
	}

	my $max = 0;
	my $val = 0;

	foreach my $subtree ( keys %{$tree} ) {
		my $child = $tree->{$subtree};

		if ( ref($child) eq "ARRAY" ) {
			foreach my $item (@$child) {
				$val = xml_find_LUT_Kvalue($item);
				if ( $val > $max ) {
					$max = $val;
				}
			}
		}
		elsif ( ref($child) eq "HASH" ) {
			$val = xml_find_LUT_Kvalue($child);
			if ( $val > $max ) {
				$max = $val;
			}
		}
		else {

			# Leaf - do nothing
		}
	}

	return $max;
}

sub xml_find_mem_size_recursive {
	my $tree = shift();

	#Check if this is a Memory
	if ( xml_find_key( $tree, "-blif_model" ) =~ "port_ram" ) {
		my $input_pins = $tree->{input};
		foreach my $input_pin (@$input_pins) {
			if ( xml_find_key( $input_pin, "-name" ) =~ "addr" ) {
				return $input_pin->{"-num_pins"};
			}
		}
		return 0;
	}

	# Otherwise iterate down
	my $max = 0;
	my $val = 0;

	foreach my $subtree ( keys %{$tree} ) {
		my $child = $tree->{$subtree};

		if ( ref($child) eq "ARRAY" ) {
			foreach my $item (@$child) {
				$val = xml_find_mem_size_recursive($item);
				if ( $val > $max ) {
					$max = $val;
				}
			}
		}
		elsif ( ref($child) eq "HASH" ) {
			$val = xml_find_mem_size_recursive($child);
			if ( $val > $max ) {
				$max = $val;
			}
		}
		else {

			# Leaf - do nothing
		}
	}

	return $max;
}

sub xml_find_mem_size {
	my $tree = shift();

	my $pb_tree = $tree->{architecture}->{complexblocklist}->{pb_type};
	if ( $pb_tree eq "" ) {
		return "";
	}

	my $memory_pb = xml_find_child_by_key_value ($pb_tree, "-name", "RAMB36E1");
	if ( $memory_pb eq "" ) {
		return "";
	}

	return xml_find_mem_size_recursive($memory_pb);
}


