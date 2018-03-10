$script_mtime = [$script_mtime, File.mtime(__FILE__).to_f].max

require_relative 'path_support.rb'

$BIN_PATH = best_path(ENV["ARME_PATH"] + "/bin", Dir.pwd) + "/"

# set this to true if you want to speed up compilation/link. This will disable the most expensive optimizations.
# the speedup will be dramatic.
$FAST_BUILD = true
# enable this if you require smaller code.
$TINY_CODE = false

$gpp_buildhandler = Class.new do
	def self.name
		return "gcc/g++ build handler"
	end
	
	def self.extensions
		return [".s", ".cpp", ".cxx", ".cc", ".c++", ".c", ".gcpp"];
	end
	
	def self.is_s(source)
		return [".s"].include? File.extname(source).downcase
	end

	def self.is_c(source)
		return [".c"].include? File.extname(source).downcase
	end

	def self.is_gccp(source)
		return [".gcpp"].include? File.extname(source).downcase
	end
	
	def self.quote_wrap(str)
		if (str =~ /\s/)
			return "\"" + str + "\""
		end
		return str
	end
	
	def self.compiler_path(source)
		return is_c(source) ? gcc_path() : gpp_path()
	end
	
	def self.gpp_path
		return quote_wrap($BIN_PATH + "arm-none-eabi-g++.exe")
	end
	
	def self.gcc_path
		return quote_wrap($BIN_PATH + "arm-none-eabi-gcc.exe")
	end
	
	def self.archiver_path
		return quote_wrap($BIN_PATH + "arm-none-eabi-gcc-ar.exe")
	end
	
	def self.objcopy_path
		return quote_wrap($BIN_PATH + "arm-none-eabi-objcopy.exe")
	end

	def self.compile_args(source = "dummy.cpp")
		buildopts = [
			is_gccp(source) ? "-x c++" : "",				# make sure the compiler is using the right language!
			is_s(source) ? "-x assembler-with-cpp" : "",
			is_c(source) ? "-std=gnu11" : "-std=gnu++17",	# set to the most recent spec the compiler can handle.
			is_c(source) ? "" : "-fno-exceptions",			# disable exception handling.
			#is_c(source) ? "" : "-fdeclone-ctor-dtor",		# enable decloning of constructor/destructors.
			is_c(source) ? "" : "-fpermissive",				# allow non-conforming code.
			is_c(source) ? "" : "-fno-threadsafe-statics",	# disable magic static code, as we don't have real threads.
			is_c(source) ? "" : "-fno-enforce-eh-specs",
			is_c(source) ? "" : "-fno-use-cxa-get-exception-ptr",
			is_c(source) ? "" : "-felide-constructors",
			"-Wno-deprecated",								# I like #include, damn it!
			"-mfloat-abi=hard",
			"-g0",											# Disable debugging information.
			"-mcpu=cortex-m4",
			"-mfpu=fpv4-sp-d16",
			is_c(source) ? "" : "-fno-rtti",
			"-march=armv7e-m",								# ARM Cortex-M
			"-mthumb",
			"-mtune=cortex-m4",								# Cortex-M3
			#"-mfp16-format=alternative",
			$TINY_CODE ? "-Os" : "-O3",						# Choose optimization level.
			#$TINY_CODE ? "-fno-tree-ch" : "",				# Disabling ftree-ch makes code smaller. Not convinced that it's _better_. TODO Validate.
			"-fno-common",
			"-funsigned-char",								# Force 'char' to be unsigned by default. Not pretty since normally types are _signed_ by default.
			"-funsigned-bitfields",							# Force bitfields to be unsigned by default.
			"-fvar-tracking-assignments",
			"-freorder-blocks",							# Disabling the reordering of basic blocks, which is supposed to limit branches. Bloats code. TODO validate faster.
			"-fmove-loop-invariants",					# Disable moving loop invariants. Makes code substantially larger. TODO validate that the code is actually better.
			"-ftree-loop-optimize",						# Disable optimization of loops. Makes code substantially larger. TODO validate that the code is actually better.
			"-ffunction-sections",						# Code generation is better with function-sections disabled.
			"-fdata-sections",							# Code generation is better with data-sections disabled.
			"-fearly-inlining",							# Coupled with the below, generates better code. The value below is hand-tuned.
			"-finline-limit=134217728",						# Presumably, it allows more to be inlined, but also enables the heuristics in more cases.
			"-flto",										# Link-Time Optimization.
			"-fno-fat-lto-objects",							# Don't generate fat object files. Our object files should only be LTO.
			"-fno-keep-static-consts",						# Elide static constant variables that are unused, and happily declone them.
			"-fmerge-all-constants",						# Merge constants that have the same value.
			"-fno-unsafe-loop-optimizations",				# TODO we have this disabled for now as it can potentially give unreliable loop results. Codegen isn't different.
			"-fpredictive-commoning",						# Re-use load/stores from previous loops.
			"-fdelete-dead-exceptions",						# Delete dead exceptions. Unlikely to do anything as exceptions are disabled.
			"-freg-struct-return",							# Prefer to return composite types from functions in registers rather than on the stack.
			"-funswitch-loops",							# Do not unswitch loops - unswitching generates worse code. TODO Investigate why. Likely GCC heuristic issue.
			"-ftree-loop-vectorize",						# Perform loop vectorization. Generates better code for some reason. TODO Investigate why, as AVR has no concept of vectored code.
			"-fsplit-paths",								# Do not split paths. Generates bloated code, but possible faster. TODO Investigate if it's faster.
			"-ftree-pre",								# PRE for some reason generates bloated code. Likely GCC bug. TODO Investigate.
			"-ftree-partial-pre",						# Making PRE more aggressive generates larger code.
			"-fgcse-sm",									# Disable performing store motion pass of subexpression elimination. Generates larger code.
			"-fgcse-las",								# Disable performing load access pass of subexpression elimination. Generates larger code.
			"-fgcse-after-reload",							# Enable load spill cleanup. Generates smaller code.
			"-fdevirtualize-speculatively",
			"-fdevirtualize-at-ltrans",
			"-free",
			"-fsched-pressure",
			"-fsched-spec-load",
			"-fipa-icf",
			"-fipa-pta",
			"-ftree-builtin-call-dce",
			"-ftree-loop-distribute-patterns",				# Loop pattern distribution. Doesn't make sense on AVR, but generates smaller code? TODO Investigate.
			"-ftree-loop-distribution",						# Loop distribution. Like above, makes no sense but generates smaller code. TODO Investigate.
			"-ftree-loop-ivcanon",
			"-fivopts",
			"-fvariable-expansion-in-unroller",
			"-fassociative-math",
			"-ffast-math",
			"-freciprocal-math",
			"-ffinite-math-only",
			"-fno-signed-zeros",
			"-fno-trapping-math",
			"-fno-math-errno",
			"-funsafe-math-optimizations",
			"-fno-fp-int-builtin-inexact",
			"-fsingle-precision-constant",
			"-fstdarg-opt",
			"-fno-strict-aliasing",							# I cannot guarantee that this code is strict-aliasing compliant. I prefer using __restrict.
			"-ffp-contract=fast",
			"-fno-keep-inline-functions",
			"-fno-keep-static-functions",
			"-fmodulo-sched-allow-regmoves",
			"-fno-unconstrained-commons",
			"-fira-hoist-pressure",
			"-fira-loop-pressure",
			"-fira-region=mixed",							# 'all' is supposed to give the smallest size, but I find that 'mixed' does.
			"-fno-semantic-interposition",
			"-fno-printf-return-value",
			"-flimit-function-alignment",
			"-fuse-linker-plugin",
			"-frename-registers",
			"-fstrong-eval-order=none",
			"-fno-trapv",
			"-fno-non-call-exceptions",
			"-fno-unwind-tables",
			"-fno-asynchronous-unwind-tables",
			"-fno-strict-volatile-bitfields",
			"-ffreestanding",
			"-fstack-check=no",
			"-fno-stack-limit",
			"-finline-small-functions",
			"-findirect-inlining",
			#"-fpack-struct",
			
			"--param max-crossjump-edges=1073741824",
			"--param min-crossjump-insns=1",
			"--param max-gcse-memory=1073741824",
			"--param max-pending-list-length=1073741824",
			"--param max-modulo-backtrack-attempts=1073741824",
			"--param max-hoist-depth=0",
			"--param max-tail-merge-comparisons=1073741824",
			"--param max-tail-merge-iterations=1073741824",
			"--param iv-consider-all-candidates-bound=1073741824",
			"--param iv-max-considered-uses=1073741824",
			$FAST_BUILD ? "" : "--param dse-max-object-size=1073741824",
			"--param scev-max-expr-size=1073741824",
			"--param scev-max-expr-complexity=1073741824",
			"--param max-iterations-to-track=1073741824",
			$FAST_BUILD ? "" : "--param max-cse-path-length=1073741824",
			"--param max-cse-insns=1073741824",
			"--param max-reload-search-insns=1073741824",
			"--param max-cselib-memory-locations=1073741824",
			"--param max-sched-ready-insns=1073741824",
			"--param max-sched-region-blocks=1073741824",
			"--param max-pipeline-region-blocks=1073741824",
			"--param max-sched-region-insns=1073741824",
			"--param max-pipeline-region-insns=1073741824",
			"--param min-spec-prob=0",
			"--param max-sched-extend-regions-iters=1073741824",
			"--param sched-mem-true-dep-cost=0",
			"--param selsched-max-lookahead=1073741824",
			"--param selsched-max-sched-times=1073741824",
			"--param selsched-insns-to-rename=1073741824",
			"--param max-last-value-rtl=1073741824",
			"--param integer-share-limit=4096",
			"--param prefetch-latency=0",
			"--param simultaneous-prefetches=0",
			"--param l1-cache-line-size=0",
			"--param l1-cache-size=0",
			"--param l2-cache-size=0",
			"--param max-partial-antic-length=1073741824",
			"--param sccvn-max-scc-size=1073741824",
			"--param sccvn-max-alias-queries-per-access=1073741824",
			"--param ira-max-loops-num=1073741824",
			"--param ira-max-conflict-table-size=1073741824",
			"--param loop-invariant-max-bbs-in-loop=1073741824",
			"--param loop-max-datarefs-for-datadeps=1073741824",
			"--param max-vartrack-size=0",
			"--param max-vartrack-expr-depth=1073741824",
			"--param ipa-max-agg-items=1073741824",
			"--param ipa-max-aa-steps=1073741824",
			"--param sink-frequency-threshold=100",
			"--param max-slsr-cand-scan=999999",
			"--param max-ssa-name-query-depth=10",
			"--param max-speculative-devirt-maydefs=1073741824",
			"--param asan-globals=0",
			"--param asan-stack=0",
			"--param asan-instrument-reads=0",
			"--param asan-instrument-writes=0",
			"--param asan-memintrin=0",
			"--param asan-use-after-return=0",
			"-I#{quote_wrap(best_path(ENV["ARME_PATH"] + "/arm-none-eabi/include", Dir.pwd))}",
			"-I.",
			"-I./Teensy",
			#"-nostdlib",
			"-MMD -D__MK66FX1M0__=1 -DTEENSYDUINO=140 -DARDUINO=10804 -DF_CPU=240000000 -DUSB_SERIAL=1 -DLAYOUT_US_ENGLISH=1",
			#"-mpure-code -D__PURE_CODE__",
			"-mfp16-format=alternative",
			"-D__FPU_PRESENT=1",
			"-munaligned-access",
			#(source.include? "mk20dx128.c") ? "-mslow-flash-data" : "", # don't generate this for the function that gets jammed in the loader.
		]

		# -fvisibility=[default|internal|hidden|protected]
		# -mint8
		# -fstrict-enums
		# -fuse-cxa-atexit

		# -mno-interrupts # use this if this were a non-ISR build. Normally, since the stack pointer is > 8 bits, interrupts are cleared/reset before
		# the stack pointer is changed as stack pointer alterations are otherwise non-atomic.
		# Obviously, the code is far faster with this set, but you cannot use interrupts.

		# -mshort-calls
		# -mcall-prologues
		
		return_opts = ""
		buildopts.each { |opt|
			if (opt.length)
				return_opts += opt + " "
			end
		}
		return return_opts.chomp
	end

	def self.link_args(library_paths, source = "dummy.cpp")
		lib_str = ""
		library_paths.each { |dir|
			lib_str += "-L#{quote_wrap(dir)} "
		}

		buildopts = [
			#"-fwhole-program",
			"-T\"Teensy/mk66fx1m0.ld\"",
			"-Wl,--defsym=__rtc_localtime=0",
			"--specs=nosys.specs",
			"-s",
			#"-Wl,-R .comment -Wl,-R .gnu.version",
			"-Wl,--gc-sections",
			"-Wl,--relax",
			"-Wl,-s",
			#"-Wl,--no-warn-mismatch",
			"#{lib_str}",
			#"-lc",
			#"-lg",
			#"-L\"C:/Program Files (x86)/GNU Tools ARM Embedded/7 2017-q4-major/arm-none-eabi/lib/hard\"",
			"-lm",
		]

		# --rodata-writable

		# We also append compiler flags to the link args.
		compile_opts = compile_args(source)

		return_opts = ""
		buildopts.each { |opt|
			if (opt.length)
				return_opts += opt + " "
			end
		}
		return compile_opts + " " + return_opts.chomp
	end
	
	def self.objcopy(command, print_cmd = true)
		command = objcopy_path + " " + command
		if (print_cmd)
			puts $TAB + command
			STDOUT.flush
		end
		
		pipe = IO.popen(command)
		output = pipe.readlines(nil)
		pipe.close
		if ($? != 0)
			raise RuntimeError.new("objcopy command failed - return code #{$?.to_s}")
		end
	end
	
	def self.compile(source, out, include_paths, print_cmd = true)		
		includes = ""
		include_paths.each { |path|
			includes += "-I#{quote_wrap(path)} "
		}
		includes.chomp!
		command = compiler_path(source) + " " + includes + " " + compile_args(source) + " -c #{quote_wrap(source)} -o #{quote_wrap(out)}"
		if (print_cmd)
			puts $TAB + command
			STDOUT.flush
		end
		
		pipe = IO.popen(command)
		pipe.close
		if ($? != 0)
			raise RuntimeError.new("Failed to compile \"#{source}\" - return code #{$?.to_s}")
		end
	end
	
	def self.get_dependencies(source, print_cmd = false)
		command = compiler_path(source) + " " + compile_args(source) + " -M -MT \"\" #{quote_wrap(source)}"
		if (print_cmd)
			puts $TAB + command
			STDOUT.flush
		end
		
		pipe = IO.popen(command)
		output = pipe.readlines(nil)
		pipe.close
		if ($? != 0)
			raise RuntimeError.new("Failed to get source dependencies for \"#{source}\" - return code #{$?.to_s}")
		end
		output = output.join(" ").strip.gsub(/\r/,"").gsub(/\n/,"").gsub(/\\ /,"++++")
		if (output.size > 1)
			output = output[2..-1].split(' ')[1..-1]
			output.map! {|filename|
				filename.gsub("++++", ' ').lstrip
			}
			clean_paths(output)

			return output.uniq
		end
		
		return [output]
	end
	
	def self.archive(archive, objects, print_cmd = true)
		@MaxCmd = 8191
	
		baseCmd = archiver_path() + " rcs " + quote_wrap(archive) + " ";
		
		commands = []
		command = baseCmd.dup
		objects.each { |object|
			object = quote_wrap(object.object_path)
			if ((command.length + object.length) >= @MaxCmd)
				# Flush command and start a new one.
				commands << command.dup.chop
				command = baseCmd.dup
			end
			command = command + object + " "
		}
		if (command != baseCmd)
			commands << command.dup.chop
		end
	
		commands.each { |command|
			if (print_cmd)
				puts $TAB + command
				STDOUT.flush
			end
			
			pipe = IO.popen(command)
			output = pipe.readlines(nil)
			pipe.close
			if ($? != 0)
				raise RuntimeError.new("Failed to archive \"#{object}\" - return code #{$?.to_s}")
			end
		}
	end
	def self.link(outfile, archive, library_paths, print_cmd = true)
		command = gcc_path() + " " + link_args(library_paths) +  "-Wl,--whole-archive \"#{archive}\" -Wl,--no-whole-archive -Wl,-Map,linker.map -o \"#{outfile}\""
		if (print_cmd)
			puts $TAB + command
			STDOUT.flush
		end
		
		pipe = IO.popen(command)
		output = pipe.readlines(nil)
		pipe.close
		if ($? != 0)
			raise RuntimeError.new("Failed to link \"#{outfile}\" - return code #{$?.to_s}")
		end
	end
end
