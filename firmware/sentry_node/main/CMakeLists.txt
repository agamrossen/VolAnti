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

find_program(SENTRY_CONDA conda
             HINTS /opt/homebrew/Caskroom/miniforge/base/bin
                   $ENV{HOME}/miniforge3/bin $ENV{HOME}/miniconda3/bin
                   /opt/homebrew/bin /usr/local/bin)
if(NOT SENTRY_CONDA)
    message(FATAL_ERROR "conda not found; the generated headers cannot be "
                        "built from the sealed Stage-0 artifacts")
endif()

# Call the env's interpreter DIRECTLY, never `conda run ... python`. Under
# `eim run` the ESP-IDF venv is already on PATH, and `conda run` resolves the
# bare name `python` against it - so the generator ran under the IDF
# interpreter and died on `import numpy`. The env python is derived from
# conda's own location, so this stays correct wherever conda is installed.
get_filename_component(SENTRY_CONDA_BIN ${SENTRY_CONDA} DIRECTORY)
set(SENTRY_PY ${SENTRY_CONDA_BIN}/../envs/acoustic-detector/bin/python)
if(NOT EXISTS ${SENTRY_PY})
    message(FATAL_ERROR "conda env 'acoustic-detector' python not found at "
                        "${SENTRY_PY}")
endif()

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
