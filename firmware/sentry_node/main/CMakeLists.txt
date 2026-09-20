idf_component_register(SRCS "sentry_node.c" "detector.c" "source.c"
                            "source_i2s.c" "trace.c"
                            # R1 and L from ONE channel, by biquad
                            # rather than a second FFT. Gates nothing;
                            # see nf_probe.h.
                            "nf_probe.c"
                            # --- alert output layer ---
                            "actuators.c" "led_ws2812.c" "led_strip_encoder.c"
                            # led_pattern.c is the state language as
                            # pure arithmetic, no ESP-IDF, so the host
                            # tests drive the SHIPPED mapping and can
                            # still measure a cadence over a period.
                            "led_pattern.c"
                            "alert_ui.c" "epaper.c"
                            # The panel is TWO files: epaper.c drives
                            # the controller, epaper_draw.c draws the
                            # pixels and contains no ESP-IDF, so
                            # tests/test_epaper_pages.py can render
                            # every screen on the host and look at it.
                            "epaper_draw.c"
                            "epaper/DEV_Config.c" "epaper/EPD_2in13_V4.c"
                            "epaper/EPD_1in54_V2.c"
                            # --- quad acquisition ---
                            "source_i2s_quad.c"
                            # --- Tier-2, additive ---
                            "detector_t2.c" "combiners_cx.c"
                            # --- Tier-3, additive AND DISABLED.
                            # Compiled in so the port is real and testable;
                            # T3_ENABLED_DEFAULT is 0 and nothing calls into
                            # it unless the `W` command turns it on.
                            "detector_t3.c"
                            # --- Tier-4, additive AND DISABLED.
                            # Same contract as Tier-3: compiled in so the port
                            # is real and testable, T4_ENABLED_DEFAULT is 0,
                            # and nothing reaches it unless it is turned on by
                            # a typed line.
                            "detector_t4.c"
                            # --- the standalone field build ---
                            "settings.c" "event_log.c"
                            # --- the RTC survivor.
                            # Boot records, live counters and
                            # an event-ring mirror in RTC_NOINIT memory,
                            # because opening a serial port on this board
                            # RESETS it and destroys the RAM ring.
                            "boot_rec.c"
                            # --- the board seam's PCB-only hardware.
                            # power_slice.c is pure integer arithmetic with no
                            # ESP-IDF in it, which is what lets the host test
                            # compile and call THE SHIPPED C rather than model
                            # it. power_mon.c is entirely inside
                            # BOARD_HAS_VBAT and compiles to a handful of
                            # inert stubs on the breadboard target.
                            "power_slice.c" "power_mon.c"
                            # lora_proto.c is the packet, the CRC and the
                            # dedupe ring - pure integers, no ESP-IDF, so
                            # tests/test_lora.py builds and drives THE SHIPPED
                            # C rather than a model of it. lora_link.c is the
                            # SX1276, the task and the REMOTE alert, entirely
                            # inside BOARD_HAS_LORA.
                            "lora_proto.c" "lora_link.c"
                            # --- mic calibration scaffolding.
                            # Pure arithmetic over accumulated spectra, no
                            # ESP-IDF, so tests/test_mic_cal.py compiles and
                            # drives THE SHIPPED C. The gains are computed and
                            # stored; applying them to the sum path needs
                            # golden + quad parity on a board first.
                            "mic_cal.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_usb_serial_jtag esp_driver_i2s esp_driver_ledc nvs_flash
                                esp_timer esp_driver_gpio esp_driver_rmt
                                esp_driver_spi esp_adc)

# --------------------------------------------------------------------------
# Float semantics. The port reproduces numpy's expression and summation order
# on purpose, so the compiler must not "improve" it:
#   -ffp-contract=off      no fused multiply-add. Xtensa MADD.S would fuse
#                          S[i]*(1-fr) + S[i+1]*fr into a single rounding and
#                          silently change the comb score in the last bits.
#   -fno-fast-math et al   no reassociation, no reciprocal substitution.
# --------------------------------------------------------------------------
target_compile_options(${COMPONENT_LIB} PRIVATE
    -ffp-contract=off
    -fno-fast-math
    -fno-unsafe-math-optimizations
    -fno-associative-math)

# Measurement hook (default OFF). SENTRY_WHITEN_F32=1 computes the whitening
# log1p in float32 instead of float64. It exists ONLY so the cycle cost of
# matching numpy's float64 whitening can be quoted; the golden result is
# always the default build.
if(DEFINED ENV{SENTRY_TABLES_LUT})
    target_compile_definitions(${COMPONENT_LIB} PRIVATE
        SENTRY_TABLES_LUT=$ENV{SENTRY_TABLES_LUT})
    message(STATUS "sentry: SENTRY_TABLES_LUT=$ENV{SENTRY_TABLES_LUT}")
endif()
if(DEFINED ENV{SENTRY_WHITEN_F32})
    target_compile_definitions(${COMPONENT_LIB} PRIVATE
        SENTRY_WHITEN_F32=$ENV{SENTRY_WHITEN_F32})
    message(STATUS "sentry: SENTRY_WHITEN_F32=$ENV{SENTRY_WHITEN_F32}")
endif()

# --------------------------------------------------------------------------
# Generated headers. NOTHING under generated/ is hand written or hand copied;
# it is emitted from data/device_config.json + src/detector.py by
# scripts/gen_headers.py, which is wired in here so a stale header cannot be
# built. The generator rewrites a file only when its bytes change (so the
# output is deterministic), and a stamp file carries the dependency edge.
# --------------------------------------------------------------------------
set(GEN_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/generated)
set(PROJ_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)
get_filename_component(REPO_DIR ${PROJ_DIR}/../.. ABSOLUTE)

# --------------------------------------------------------------------------
# The interpreter that runs the generator. What it has to be is a Python 3
# with numpy and nothing else: gen_headers.py imports numpy, src/detector.py
# and src/operating_point.py, and those import numpy and the standard
# library.
#
# So every candidate is ASKED whether it can import numpy instead of being
# trusted for where it came from, and that is what makes it safe to look at
# PATH. Under `idf.py` the ESP-IDF virtualenv is first on PATH and has no
# numpy, so a search that went by name would hand the generator an
# interpreter that dies on `import numpy` part way through the build - the
# same trap `conda run ... python` fell into. Asking rejects it and moves on.
#
# In order: SENTRY_PYTHON, the conda environment from environment.yml, an
# active conda environment, python3 and python from PATH, and the Windows py
# launcher. Which one wins does not change what comes out: 3.11 with numpy
# 2.4.6, 3.14 with 2.5.3 and 3.10 with 1.26.4 emit the same eleven headers,
# byte for byte.
# --------------------------------------------------------------------------

# TRUE when this interpreter satisfies the generator. 3.7 is the floor
# because src/detector.py uses dataclasses, and numpy is the whole of the
# dependency: there is no import from outside the standard library anywhere
# else on the generator's path.
function(sentry_python_ok py out)
    set(${out} FALSE PARENT_SCOPE)
    if(NOT EXISTS "${py}")
        return()
    endif()
    execute_process(RESULT_VARIABLE sentry_rc OUTPUT_QUIET ERROR_QUIET
        COMMAND "${py}" -c
        "import sys, numpy; sys.exit(0 if sys.version_info >= (3, 7) else 1)")
    if(sentry_rc STREQUAL "0")
        set(${out} TRUE PARENT_SCOPE)
    endif()
endfunction()

# An environment keeps its interpreter in bin/ on macOS and Linux, and at the
# top or in Scripts/ on Windows. All three spellings go into the list and
# EXISTS decides, so the search does not rest on detecting the platform.
macro(sentry_add_env_prefix prefix)
    list(APPEND SENTRY_CANDIDATES "${prefix}/bin/python"
                                  "${prefix}/python.exe"
                                  "${prefix}/Scripts/python.exe")
endmacro()

if(DEFINED ENV{SENTRY_PYTHON})
    set(SENTRY_PYTHON "$ENV{SENTRY_PYTHON}" CACHE FILEPATH
        "Python 3 with numpy for scripts/gen_headers.py" FORCE)
endif()

set(SENTRY_PY "")
set(SENTRY_REJECTED "")

if(SENTRY_PYTHON)
    # Named outright, so it has to work, and saying why here beats a
    # traceback out of the generator part way through the build.
    file(TO_CMAKE_PATH "${SENTRY_PYTHON}" SENTRY_PY)
    if(NOT EXISTS "${SENTRY_PY}")
        find_program(SENTRY_PY_ON_PATH NAMES "${SENTRY_PY}" NO_CACHE)
        if(SENTRY_PY_ON_PATH)
            set(SENTRY_PY "${SENTRY_PY_ON_PATH}")
        else()
            message(FATAL_ERROR "SENTRY_PYTHON is ${SENTRY_PYTHON}, which is "
                                "neither a file nor a program on PATH")
        endif()
    endif()
    sentry_python_ok("${SENTRY_PY}" SENTRY_PY_WORKS)
    if(NOT SENTRY_PY_WORKS)
        message(FATAL_ERROR
            "SENTRY_PYTHON is ${SENTRY_PYTHON}, which is not a Python 3.7 or "
            "newer that can import numpy. Install numpy into it with\n"
            "      \"${SENTRY_PY}\" -m pip install numpy")
    endif()
else()
    set(SENTRY_CANDIDATES "")

    # The environment environment.yml creates, wherever conda put it. From
    # conda's own location where the shell has conda in it, and from the
    # default install roots where it does not - the Windows installer leaves
    # conda off PATH, so a build started from VS Code or an ESP-IDF terminal
    # usually cannot see it at all.
    set(SENTRY_CONDA_ROOTS "")
    if(DEFINED ENV{CONDA_EXE})
        get_filename_component(SENTRY_DIR "$ENV{CONDA_EXE}" DIRECTORY)
        get_filename_component(SENTRY_DIR "${SENTRY_DIR}" DIRECTORY)
        list(APPEND SENTRY_CONDA_ROOTS "${SENTRY_DIR}")
    endif()
    if(DEFINED ENV{CONDA_PREFIX})
        # The base environment, or an environment under <base>/envs.
        get_filename_component(SENTRY_DIR "$ENV{CONDA_PREFIX}/../.." ABSOLUTE)
        list(APPEND SENTRY_CONDA_ROOTS "$ENV{CONDA_PREFIX}" "${SENTRY_DIR}")
    endif()
    find_program(SENTRY_CONDA NAMES conda conda.exe conda.bat NO_CACHE)
    if(SENTRY_CONDA)
        # What is on PATH is usually a link or a condabin shim; the
        # environments sit next to the real one.
        get_filename_component(SENTRY_DIR "${SENTRY_CONDA}" REALPATH)
        get_filename_component(SENTRY_DIR "${SENTRY_DIR}" DIRECTORY)
        get_filename_component(SENTRY_DIR "${SENTRY_DIR}" DIRECTORY)
        list(APPEND SENTRY_CONDA_ROOTS "${SENTRY_DIR}")
    endif()
    foreach(SENTRY_HOME "$ENV{HOME}" "$ENV{USERPROFILE}")
        if(SENTRY_HOME)
            list(APPEND SENTRY_CONDA_ROOTS "${SENTRY_HOME}/miniforge3"
                 "${SENTRY_HOME}/miniconda3" "${SENTRY_HOME}/anaconda3"
                 "${SENTRY_HOME}/mambaforge" "${SENTRY_HOME}/.conda")
        endif()
    endforeach()
    list(APPEND SENTRY_CONDA_ROOTS /opt/conda /opt/miniconda3
         /opt/homebrew/Caskroom/miniforge/base
         C:/ProgramData/miniconda3 C:/ProgramData/anaconda3)
    foreach(SENTRY_ROOT IN LISTS SENTRY_CONDA_ROOTS)
        sentry_add_env_prefix("${SENTRY_ROOT}/envs/acoustic-detector")
    endforeach()

    # An environment that is already active, whatever it is called.
    if(DEFINED ENV{CONDA_PREFIX})
        sentry_add_env_prefix("$ENV{CONDA_PREFIX}")
    endif()

    # Then whatever Python 3 the machine has. PATH is walked entry by
    # entry rather than handed to find_program, because find_program stops
    # at the first match and under `idf.py` the first python on PATH is the
    # ESP-IDF virtualenv: one numpy-less interpreter at the front would
    # otherwise hide every good one behind it. TO_CMAKE_PATH splits a search
    # path on : or ; as the platform requires.
    file(TO_CMAKE_PATH "$ENV{PATH}" SENTRY_PATH_DIRS)
    foreach(SENTRY_DIR IN LISTS SENTRY_PATH_DIRS)
        list(APPEND SENTRY_CANDIDATES "${SENTRY_DIR}/python3"
             "${SENTRY_DIR}/python" "${SENTRY_DIR}/python3.exe"
             "${SENTRY_DIR}/python.exe")
    endforeach()
    find_program(SENTRY_PY3 NAMES python3 NO_CACHE)
    if(SENTRY_PY3)
        list(APPEND SENTRY_CANDIDATES "${SENTRY_PY3}")
    endif()
    find_program(SENTRY_PY_ANY NAMES python NO_CACHE)
    if(SENTRY_PY_ANY)
        list(APPEND SENTRY_CANDIDATES "${SENTRY_PY_ANY}")
    endif()

    list(REMOVE_DUPLICATES SENTRY_CANDIDATES)
    foreach(SENTRY_CAND IN LISTS SENTRY_CANDIDATES)
        sentry_python_ok("${SENTRY_CAND}" SENTRY_CAND_WORKS)
        if(SENTRY_CAND_WORKS)
            set(SENTRY_PY "${SENTRY_CAND}")
            break()
        elseif(EXISTS "${SENTRY_CAND}")
            list(APPEND SENTRY_REJECTED "${SENTRY_CAND}")
        endif()
    endforeach()

    # The Windows launcher is not an interpreter, it picks one, so it cannot
    # go in the list above. python.org's installer leaves py on PATH even
    # when it does not put Python itself there, which is the common case.
    if(NOT SENTRY_PY)
        find_program(SENTRY_PY_LAUNCHER NAMES py NO_CACHE)
        if(SENTRY_PY_LAUNCHER)
            execute_process(RESULT_VARIABLE sentry_rc ERROR_QUIET
                OUTPUT_VARIABLE SENTRY_LAUNCHED
                COMMAND "${SENTRY_PY_LAUNCHER}" -3 -c
                "import sys; sys.stdout.write(sys.executable)")
            string(STRIP "${SENTRY_LAUNCHED}" SENTRY_LAUNCHED)
            if(sentry_rc STREQUAL "0" AND SENTRY_LAUNCHED)
                file(TO_CMAKE_PATH "${SENTRY_LAUNCHED}" SENTRY_LAUNCHED)
                sentry_python_ok("${SENTRY_LAUNCHED}" SENTRY_CAND_WORKS)
                if(SENTRY_CAND_WORKS)
                    set(SENTRY_PY "${SENTRY_LAUNCHED}")
                elseif(EXISTS "${SENTRY_LAUNCHED}")
                    list(APPEND SENTRY_REJECTED "${SENTRY_LAUNCHED}")
                endif()
            endif()
        endif()
    endif()
endif()

if(NOT SENTRY_PY)
    set(SENTRY_TRIED "")
    if(SENTRY_REJECTED)
        list(REMOVE_DUPLICATES SENTRY_REJECTED)
        string(REPLACE ";" "\n      " SENTRY_TRIED "${SENTRY_REJECTED}")
        set(SENTRY_TRIED
            "\n  Found, but without numpy:\n      ${SENTRY_TRIED}")
    endif()
    message(FATAL_ERROR
        "no Python 3 with numpy was found. The build needs one because it "
        "generates main/generated/ from src/ and data/ rather than keeping "
        "the headers in the repository.${SENTRY_TRIED}\n"
        "  Add numpy to a Python you already have:\n"
        "      python -m pip install numpy\n"
        "      py -m pip install numpy   (Windows)\n"
        "  or create the environment from environment.yml at the top of the "
        "repository:\n"
        "      conda env create -f environment.yml\n"
        "  or name an interpreter yourself:\n"
        "      idf.py -DSENTRY_PYTHON=<path to python> build")
endif()
message(STATUS "sentry: generator python: ${SENTRY_PY}")

set(GEN_HEADERS
    ${GEN_DIR}/detector_config.h
    ${GEN_DIR}/window.h
    ${GEN_DIR}/tables.h
    ${GEN_DIR}/t2_config.h
    ${GEN_DIR}/t3_config.h
    ${GEN_DIR}/t4_config.h
    ${GEN_DIR}/vectors.h
    ${GEN_DIR}/golden_strong_pos.h
    ${GEN_DIR}/golden_marginal_neg.h
    ${GEN_DIR}/golden_marginal_pos.h)

add_custom_command(
    OUTPUT    ${GEN_DIR}/.gen.stamp
    BYPRODUCTS ${GEN_HEADERS}
    COMMAND   ${SENTRY_PY} ${PROJ_DIR}/scripts/gen_headers.py --out ${GEN_DIR}
    COMMAND   ${CMAKE_COMMAND} -E touch ${GEN_DIR}/.gen.stamp
    DEPENDS   ${PROJ_DIR}/scripts/gen_headers.py
              ${REPO_DIR}/data/device_config.json
              ${REPO_DIR}/data/t2_config.json
              ${REPO_DIR}/data/t3_config.json
              ${REPO_DIR}/data/t4_config.json
              ${REPO_DIR}/src/detector_t2.py
              ${REPO_DIR}/src/detector_t3.py
              ${REPO_DIR}/src/detector_t4.py
              ${REPO_DIR}/data/golden_vectors.json
              ${REPO_DIR}/src/detector.py
              ${REPO_DIR}/src/operating_point.py
              ${REPO_DIR}/data/golden_strong_pos.h
              ${REPO_DIR}/data/golden_marginal_neg.h
              ${REPO_DIR}/data/golden_marginal_pos.h
    COMMENT   "gen_headers.py -> main/generated (from the sealed artifacts)"
    VERBATIM)

add_custom_target(sentry_generated DEPENDS ${GEN_DIR}/.gen.stamp)
add_dependencies(${COMPONENT_LIB} sentry_generated)
