################################################################################
# Options                                                                      #
################################################################################
OPTION(BUILD_UNITTESTS "Building Unit-Tests" ON)
OPTION(BUILD_SHARED_LIBS "Build shared libraries." ON)
OPTION(BUILD_API_DOCUMENTATION "Build API documentation (Doxygen)" ON)

################################################################################
# Config                                                                       #
################################################################################
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
SET(THIRD_PARTY_DIR ${PROJECT_BINARY_DIR}/3psw)
