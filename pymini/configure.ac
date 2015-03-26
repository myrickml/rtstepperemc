#
# configure.ac - autoconf input file rtstepperemc
# 
# (c) 2010-2015 Copyright Eckler Software
# 
# Author: David Suffield, dsuffiel@ecklersoft.com
#
# Notes on cross-compiling with mingw32.
#
#  1. Install mingw.
#
#     yum install mingw32-*
#
#  2. Now build binary for Windows.
#
#     ./bootstrap
#     ./configure --host i686-pc-mingw32 CFLAGS=-I/usr/i686-pc-mingw32/sys-root/mingw/include/ddk (Fedora 13)
#
#     or
#
#     ./configure --host i586-mingw32msvc (Ubuntu 9.04)
#     ./configure --host=i686-w64-mingw32 LDFLAGS="-L/usr/i686-w64-mingw32/lib" (Ubuntu 12.04 32-bit)
#     ./configure --host=x86_64-w64-mingw32 LDFLAGS="-L/usr/x86_64-w64-mingw32/lib" (Ubuntu 12.04 64-bit)
#
#     make
#     make zip
#
#  3. Copy the zip file to Windows XP and unzip. 
#
# Notes on compiling on OSX.
#
#  1. Download install xcode to get the GNU tools.
#  2. Download install macports at www.macports.org.
#  3. Install libusb support
#
#      sudo port install libusb-compat
#
#  4. Build binary for OSX 10.9
#
#     ./configure --prefix=/opt/local LDFLAGS=-L/opt/local/lib CFLAGS="-g -Wall -I/opt/local/include -I/opt/local/include/libusb-1.0" CXXFLAGS="-g -Wall -I/opt/local/include/libusb-1.0"
#     make
#     make dmg 
#
# Notes on compiling on Ubuntu.
#
#     ./configure --prefix=/usr CFLAGS="-g -Wall -I/usr/include/libusb-1.0" CXXFLAGS="-g -Wall -I/usr/include/libusb-1.0" (Ubuntu 12.04)
#
#     make
#     make install
#
# Handy C format command:
#     indent -bli0 -nbad -i3 -npcs -npsl -nut -l160 -c1 tclext.c
#
# History:
#
# Version 0.1,  des 12/17/2014    Rewrite using python gui.
# Version 0.6,  des 12/29/2014    Interpreter compiles.
# Version 0.8,  des 01/02/2015    Interpreter mdi commands work ok.
# Version 0.9,  des 01/07/2015    Unfinished version with tp from emc2-2.4.4.
# Version 0.10, des 01/07/2015    Compiles with tp from emc-2.1.
# Version 0.11, des 01/07/2015    emc-2.1 tp works ok.
# Version 0.15, des 01/21/2015    Rtstepper io works ok with mdi command. Uses a timeout in rtstepper_wait_xfr().
# Version 0.16, des 01/23/2015    Basic gui_event_cb() works ok.
# Version 0.17, des 01/24/2015    Mdi commands works ok with dongle only.
# Version 0.19, des 01/24/2015    Mdi commands work on Sherline 2000 CNC.
# Version 0.23, des 01/30/2015    Leadscrew compensation (backlash) now works ok. 
# Version 0.25, des 02/02/2015    Initial commands estop and auto are working.
# Version 0.29, des 02/06/2015    All Zero works, added status bar line counts and soft limits. 
# Version 0.31, des 02/08/2015    Renamed package to pymini. Application '.' directory works.
# Version 0.32, des 02/09/2015    Python plugin works for m3.py. Remove post_idle_cb() from dll.
# Version 0.33, des 02/10/2015    Converted pyemc.py to a class.
# Version 0.34, des 02/11/2015    Input abort code is ready. Fixed a event_thread() estop hang.
# Version 0.36, des 02/15/2015    Fixed intermittent estop hangs.  
# Version 0.38, des 02/16/2015    Made compatible changes for python2.7 and python3.3. Builds and runs on OSX 10.9.
# Version 0.39, des 02/24/2015    Added Arduino Uno support for IO via plugins.
# Version 0.41, des 03/01/2015    Initial backplot works.
# Version 0.60, des 03/04/2015    Added A axis support. 
# Version 0.64, des 03/15/2015    Added Verify support.
# Version 0.65, des 03/16/2015    Made Mac OSX compatable gui changes.
# Version 0.69, des 03/25/2015    Added an icon. Fixed a step pulse bug in rtstepper_start_xfr().

AC_INIT([pymini], [0.69])
AC_CANONICAL_HOST

# Checks for programs.
AC_PROG_CC
AC_PROG_CXX
AC_PROG_INSTALL
AC_GNU_SOURCE
#AC_C_BIGENDIAN

darwin_build="no"
mingw_build="no"
linux_build="no"

AC_MSG_CHECKING("for canonical host")
case "$host" in
   *-darwin*)
      darwin_build="yes"
      AC_MSG_RESULT("OS X")
      ;;

   *-mingw*)
      mingw_build="yes"
      AC_MSG_RESULT("Windows cross compile")
      ;;

     *)
      linux_build="yes"
      AC_MSG_RESULT("Linux/Unix")
      ;;
esac
AM_CONDITIONAL(DARWIN_BUILD, test x$darwin_build = xyes)

AC_MSG_CHECKING([for mini build])
AC_ARG_ENABLE(mini_build,
  [  --enable-mini-build    enable mini gui build (default=yes)],
  mini_build=$enableval, mini_build=yes)
if test "$mini_build" = "yes"; then
   AC_MSG_RESULT(yes)
else
   AC_MSG_RESULT(no)
fi
AM_CONDITIONAL(MINI_BUILD, test x$mini_build = xyes)

# Checks for required common libraries.
AC_CHECK_LIB([m], [pow],, [AC_MSG_ERROR([cannot find libm math support], 10)])

# Checks for required development packages.
AC_CHECK_HEADERS(pthread.h,, [AC_MSG_ERROR([cannot find libpthread-devel support], 11)])
#AC_CHECK_HEADERS(curses.h,, [AC_MSG_ERROR([cannot find libcurses-devel support], 12)])
if test "$mingw_build" = "yes"; then
   AC_CHECK_HEADERS(usb-winusb.h,, [AC_MSG_ERROR([cannot find libusb-winusb support], 13)])
else
   AC_CHECK_HEADERS(libusb.h,, [AC_MSG_ERROR([cannot find libusb-devel 1.0 support], 13)])
fi

AC_MSG_CHECKING(for host machine platform)
machine=`uname -m`
if test "$machine" = "x86_32" || test "$machine" = "i686"; then
    platform="x86_32"
elif test "$machine" = "x86_64"; then
    platform="x86_64"
elif test "$machine" = "ppc"; then
    platform="ppc"
else
    platform="unsupport"
fi
AC_MSG_RESULT($platform)

AC_CHECK_FUNCS([strcasestr])
AC_CHECK_FUNCS([sincos])

AC_ARG_WITH(relnum, AC_HELP_STRING([--with-relnum=N], [set release number (default=0)]),
   relnum=$withval, relnum=0)

AC_ARG_WITH(rulesdir, AC_HELP_STRING([--with-rulesdir=DIR], [set udev rules directory [default=/etc/udev/rules.d]]),
   rulesdir=$withval, rulesdir="/etc/udev/rules.d")

if test "$mingw_build" = "yes"; then
   dll_extension="dll"
   if test "$host" = "i686-pc-mingw32"; then
      mingw_path="/usr/i686-pc-mingw32/sys-root/mingw/bin"
      mingw_inc_path="/usr/i686-pc-mingw32/sys-root/mingw/include"
      mingw_lib_path="/usr/i686-pc-mingw32/sys-root/mingw/lib"
   elif test "$host" = "i686-w64-mingw32"; then
      mingw_path="/usr/${host}/lib"
      mingw_inc_path="/usr/${host}/include"
      mingw_lib_path="/usr/${host}/lib"  
   elif test "$host" = "x86_64-w64-mingw32"; then
      mingw_path="/usr/${host}/lib"
      mingw_inc_path="/usr/${host}/include"
      mingw_lib_path="/usr/${host}/lib"  
   else
      mingw_path="/usr/i586-mingw32msvc/lib"
      mingw_inc_path="/usr/i586-mingw32msvc/include" # ??
      mingw_lib_path="/usr/i586-mingw32msvc/lib"
   fi
elif test "$darwin_build" = "yes"; then
   dll_extension="dylib"
else
   dll_extension="so"
   CFLAGS="$CFLAGS -fPIC"
   CXXFLAGS="$CXXFLAGS -fPIC"
fi

AC_DEFUN([AC_DEFINE_DIR], [
  prefix_NONE=
  exec_prefix_NONE=
  test "x$prefix" = xNONE && prefix_NONE=yes && prefix=$ac_default_prefix
  test "x$exec_prefix" = xNONE && exec_prefix_NONE=yes && exec_prefix=$prefix
dnl In Autoconf 2.60, ${datadir} refers to ${datarootdir}, which in turn
dnl refers to ${prefix}.  Thus we have to use `eval' twice.
  eval ac_define_dir="\"[$]$2\""
  eval ac_define_dir="\"$ac_define_dir\""
  $1="$ac_define_dir"
  test "$prefix_NONE" && prefix=NONE
  test "$exec_prefix_NONE" && exec_prefix=NONE
])

AC_DEFINE_DIR([abs_bindir], [bindir])
AC_DEFINE_DIR([abs_datarootdir], [datarootdir])

AC_SUBST(platform)
AC_SUBST(relnum)
AC_SUBST(rulesdir)
AC_SUBST(dll_extension)
AC_SUBST(mingw_path)
AC_SUBST(mingw_inc_path)
AC_SUBST(mingw_lib_path)
AC_SUBST(mingw_build)
AC_SUBST(mini_build)
AC_SUBST(darwin_build)
AC_SUBST(linux_build)

AC_CONFIG_FILES(Makefile)
AC_OUTPUT
