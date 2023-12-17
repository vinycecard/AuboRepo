# NSIS installer script for the Pololu AVR C/C++ Library.
# Detects WinAVR and various versions of AVR/Atmel studio and adds the
# library's .h and .a files in the appropriate directories.
#
# When "make zip" is run, installer.nsi.template is edited slightly, saved
# as installer.nsi, and added to the library zip file
# (e.g. libpololu-avr-120209.zip).
# To compile the Windows NSIS installer, unzip the zip file,
# right click on installer.nsi, and select "Compile NSIS Script".
# You will need NSIS 2.46+ installed.

!define LIB_VER "150324"

!define libpololu-avr "."

SetCompressor /solid lzma

!include LogicLib.nsh
!include nsDialogs.nsh
!include FileFunc.nsh

RequestExecutionLevel admin
OutFile "libpololu-avr-${LIB_VER}.exe"
Name "Pololu AVR C/C++ Library"
InstallDir "C:\libpololu-avr"
ShowInstDetails show
AllowSkipFiles on
XPStyle on

Page directory "" "" checkDirectory
Page custom optionsPage optionsPageLeave
Page instfiles

Var /GLOBAL VSShellPath
Var Dialog

DirText "Setup will install the Pololu AVR C/C++ Library (version ${LIB_VER}) in the following folder.  To install in a different folder, click Browse and select another folder.  Click Next to continue."

# Toolchain variables
Var WinAVRPath
Var AS51Path
Var AS60Path
Var AS61Path
Var AS61PathAvr
Var AS62Path
Var AS62PathAvr

Var WinAVRBox
Var AS51Box
Var AS60Box
Var AS61Box
Var AS62Box

Var WinAVRIntegrate
Var AS51Integrate
Var AS60Integrate
Var AS61Integrate
Var AS62Integrate

# When the installer starts, detect the different things we need to detect.
Function .onInit
    Call WinAVRCheck
    Call AS51Check
    Call AS60Check
    Call AS61Check
    Call AS62Check
    Call VSShellCheck

    # Check the assumptions we make when calling NSD_GetState and NSD_SetState.
    ${If} ${BST_CHECKED} != 1
      Abort
    ${EndIf}
    ${If} ${BST_UNCHECKED} != 0
      Abort
    ${EndIf}

    # By default, any checkbox that is available will be checked.
    StrCpy $WinAVRIntegrate "1"
    StrCpy $AS51Integrate "1"
    StrCpy $AS60Integrate "1"
    StrCpy $AS61Integrate "1"
    StrCpy $AS62Integrate "1"
FunctionEnd

!macro CreateToolchainCheckBox Name Path Checked Box Y
    ${If} "${Path}" != ""
      StrCpy $1 "${Name} (${Path})"
      StrCpy $2 1                    # enable the checkbox
      StrCpy $3 "${Checked}"         # check the box if appropriate
    ${Else}
      StrCpy $1 "${Name} (not detected)"
      StrCpy $2 0                    # disable the checkbox
      StrCpy $3 0                    # uncheck the checkbox
    ${Endif}
    ${NSD_CreateCheckBox} 0 ${Y} 100% 12u $1
    Pop ${Box}
    EnableWindow ${Box} $2     # disable/enable the checkbox
    ${NSD_SetState} ${Box} $3
!macroend

# Creates the optionsPage and all the checkboxes in it.
Function optionsPage
    nsDialogs::Create 1018
    Pop $Dialog
    ${If} $Dialog == error
        Abort
    ${EndIf}

    GetFunctionAddress $0 "optionsPageLeave"
    nsDialogs::OnBack $0

    ${NSD_CreateLabel} 0 0 100% 12u "Install the library into the following toolchains:"
    Pop $0

    !insertmacro CreateToolchainCheckBox "WinAVR" $WinAVRPath $WinAVRIntegrate $WinAVRBox 20u
    !insertmacro CreateToolchainCheckBox "AVR Studio 5.1" $AS51Path $AS51Integrate $AS51Box 40u
    !insertmacro CreateToolchainCheckBox "Atmel Studio 6.0" $AS60Path $AS60Integrate $AS60Box 60u
    !insertmacro CreateToolchainCheckBox "Atmel Studio 6.1" $AS61Path $AS61Integrate $AS61Box 80u
    !insertmacro CreateToolchainCheckBox "Atmel Studio 6.2" $AS62Path $AS62Integrate $AS62Box 100u

    nsDialogs::Show
FunctionEnd

# Gets called when leaving the options page (forward or back)
# Stores the user's choices from the check boxes.
Function optionsPageLeave
  ${NSD_GetState} $WinAVRBox $WinAVRIntegrate
  ${NSD_GetState} $AS51Box $AS51Integrate
  ${NSD_GetState} $AS60Box $AS60Integrate
  ${NSD_GetState} $AS61Box $AS61Integrate
  ${NSD_GetState} $AS62Box $AS62Integrate
FunctionEnd

# Install the header (.h) and archive (.a) files that allow
# compilation of programs that use the library.
# We check the existence of the directories here because the toolchain location for
# AVR Studio 5.1 looks like it will change often and we'd like to give useful error
# messages.
!macro InstallIntoToolChain AvrPath
    ${if} ${FileExists} "${AvrPath}\lib"
        SetOutPath "${AvrPath}\lib"
        File "${libpololu-avr}\libpololu_*.a"
    ${else}
        DetailPrint "AVR lib folder not found: ${AvrPath}\lib"
        MessageBox MB_OK|MB_ICONEXCLAMATION "The folder ${AvrPath}\lib was not found.  You might have to find the correct lib folder for your toolchain and copy the $INSTDIR\libpololu_*.a files to it manually.  Please report this problem at http://forum.pololu.com/."
    ${endif}

    ${if} ${FileExists} "${AvrPath}\include"
        SetOutPath "${AvrPath}\include"
        File /r "${libpololu-avr}\pololu"
    ${else}
        DetailPrint "AVR include folder not found: ${AvrPath}\include"
        MessageBox MB_OK|MB_ICONEXCLAMATION "The folder ${AvrPath}\include was not found.  You might have to find the correct include folder for your toolchain and copy the $INSTDIR\pololu folder to it manually.  Please report this problem at http://forum.pololu.com/."
    ${endif}
!macroend

!macro InstallSTK500Xml Path
    # Install the STK500 XML files needed for programming, but don't
    # overwrite any existing XML files.
    SetOutPath "${Path}"
    SetOverwrite off
    File "${libpololu-avr}\avr_studio_5\stk500_xml\*.xml"
    SetOverwrite on
!macroend

Section
    SetOutPath "$INSTDIR"
    File /r /x libpololu-avr-*.exe "${libpololu-avr}\*.*"

    StrCmp $WinAVRPath "" end_winavr
    IntCmp $WinAVRIntegrate 0 end_winavr
    DetailPrint "Installing library into WinAVR..."
    !insertmacro InstallIntoToolChain "$WinAVRPath\avr"
    end_winavr:

    StrCmp $AS51Path "" end_as51
    IntCmp $AS51Integrate 0 end_as51
    DetailPrint "Installing library into AVR Studio 5.1..."
    ; NOTE: For the AVR Studio 5.1 beta, this path didn't have the ".27" in it.
    !insertmacro InstallIntoToolChain "$AS51Path\extensions\Atmel\AVRGCC\3.3.1.27\AVRToolChain\avr"
    !insertmacro InstallSTK500Xml "$AS51Path\tools\STK500\xml"
    ExecWait "$\"$VSShellPath\Common7\IDE\vsixinstaller.exe$\" /skuName:AvrStudio /skuVersion:5.1 /quiet $\"$INSTDIR\avr_studio_5\extension.vsix$\""
    end_as51:

    StrCmp $AS60Path "" end_as60
    IntCmp $AS60Integrate 0 end_as60
    DetailPrint "Installing library into Atmel Studio 6.0..."
    ${if} ${FileExists} "$AS60Path\extensions\Atmel\AVRGCC\3.4.1.95\AVRToolchain\avr"
      DetailPrint "Detect Atmel Studio 6.0 build 1996" # released November 2012
      !insertmacro InstallIntoToolChain "$AS60Path\extensions\Atmel\AVRGCC\3.4.1.95\AVRToolchain\avr"
    ${elseif} ${FileExists} "$AS60Path\extensions\Atmel\AVRGCC\3.4.0.65\AVRToolchain\avr"
      DetailPrint "Detect Atmel Studio 6.0 build 1843"
      !insertmacro InstallIntoToolChain "$AS60Path\extensions\Atmel\AVRGCC\3.4.0.65\AVRToolchain\avr"
    ${else}
      DetailPrint "Unable to detect Atmel Studio 6.0 build version."
      MessageBox MB_OK|MB_ICONEXCLAMATION "The Pololu AVR C/C++ Library installer was unable to find the avr folder for Atmel Studio 6.0's AVR toolchain.  You might have to find your AVR toolchain's avr folder (probably inside $AS60Path\extensions\Atmel\AVRGCC\ ) and then manually copy the $INSTDIR\libpololu_*.a files to avr\lib and copy the $INSTDIR\pololu folder to avr\include.  Please report this problem at http://forum.pololu.com/."
    ${endif}
    !insertmacro InstallSTK500Xml "$AS60Path\tools\STK500\xml"
    ExecWait "$\"$VSShellPath\Common7\IDE\vsixinstaller.exe$\" /skuName:AtmelStudio /skuVersion:6.0 /quiet $\"$INSTDIR\avr_studio_5\extension.vsix$\""
    end_as60:

    StrCmp $AS61Path "" end_as61
    IntCmp $AS61Integrate 0 end_as61
    DetailPrint "Installing library into Atmel Studio 6.1..."
    ${if} ${FileExists} $AS61PathAvr
      !insertmacro InstallIntoToolChain $AS61PathAvr
    ${else}
      DetailPrint "Unable to find Atmel Studio 6.1 toolchain.  Looked in $AS61PathAvr"
      MessageBox MB_OK|MB_ICONEXCLAMATION "The Pololu AVR C/C++ Library installer was unable to find the avr folder for Atmel Studio 6.1's 8-bit GNU toolchain.  You might have to find your AVR toolchain's avr folder (probably near $AS61Path ) and then manually copy the $INSTDIR\libpololu_*.a files to avr\lib and copy the $INSTDIR\pololu folder to avr\include.  Please report this problem at http://forum.pololu.com/."
    ${endif}
    !insertmacro InstallSTK500Xml "$AS61Path\tools\STK500\xml"
    ExecWait "$\"$VSShellPath\Common7\IDE\vsixinstaller.exe$\" /skuName:AtmelStudio /skuVersion:6.1 /quiet $\"$INSTDIR\avr_studio_5\extension.vsix$\""
    end_as61:

    StrCmp $AS62Path "" end_as62
    IntCmp $AS62Integrate 0 end_as62
    DetailPrint "Installing library into Atmel Studio 6.2..."
    ${if} ${FileExists} $AS62PathAvr
      !insertmacro InstallIntoToolChain $AS62PathAvr
    ${else}
      DetailPrint "Unable to find Atmel Studio 6.2 toolchain.  Looked in $AS62PathAvr"
      MessageBox MB_OK|MB_ICONEXCLAMATION "The Pololu AVR C/C++ Library installer was unable to find the avr folder for Atmel Studio 6.2's 8-bit GNU toolchain.  You might have to find your AVR toolchain's avr folder (probably near $AS62Path ) and then manually copy the $INSTDIR\libpololu_*.a files to avr\lib and copy the $INSTDIR\pololu folder to avr\include.  Please report this problem at http://forum.pololu.com/."
    ${endif}
    !insertmacro InstallSTK500Xml "$AS62Path\tools\STK500\xml"
    ExecWait "$\"$VSShellPath\Common7\IDE\vsixinstaller.exe$\" /skuName:AtmelStudio /skuVersion:6.2 /quiet $\"$INSTDIR\avr_studio_5\extension.vsix$\""
    end_as62:

SectionEnd

# Checks to see if WinAVR is installed.
# Sets $WinAVRPath to the path or "" if not found.
Function WinAVRCheck
    ReadRegStr $0 HKLM "SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\WinAVR" 'UninstallString'
    ${GetParent} $0 $WinAVRPath
FunctionEnd

# The location of AVR Studio 5.1 is stored in several places:
# HKLM "SOFTWARE\Microsoft\AppEnv\10.0\Apps\AVRStudio_5.1" 'StubExePath' (requires GetParent, seems to get deleted when AVR Studio 5.0 is uninstalled)
# HKLM "SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\{D574D18C-9D52-4B4B-9647-AE6B89FD3F70}" 'InstallLocation'
# HKLM "SOFTWARE\Microsoft\Windows\CurrentVersion\App Paths\avrstudio.exe" (Default) (requires GetParent)
# HKCU "SOFTWARE\Atmel\AVRStudio51\5.1_Config" 'InstallDir'
#
# We would like to look in HKLM (HKEY_LOCAL_MACHINE) instead of HKCU (HKEY_CURRENT_USER) because
# it doesn't really make sense to look at user settings to find where an application installed.
# We would also like to use a registry key that appears to be used consistently between the
# different versions of AVR Studio 5 so it is easy to update our installers.
# We would also like one that is predictable (e.g. doesn't have a GUID in it).
# The only one that satisfies all of those criteria is the AppEnv one, so we will use that.
# BUT David noticed that it disappear from his computer after uninstalling AVR Studio 5.0, so
# we will use the Uninstall key as a fallback.
# Checks to see if AVR Studio 5 is installed.
# Sets $AS51Path to the location or "" if not found.
Function AS51Check
    ReadRegStr $0 HKLM "SOFTWARE\Microsoft\AppEnv\10.0\Apps\AVRStudio_5.1" 'StubExePath'
    ${GetParent} $0 $AS51Path
    StrCmp $AS51Path "" 0 AS51Done    # If we found it, then we are done.

    ReadRegStr $AS51Path HKLM "SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\{D574D18C-9D52-4B4B-9647-AE6B89FD3F70}" 'InstallLocation'
    StrCmp $AS51Path "" AS51Done      # If AVR Studio 5.1 really isn't there, we are done.

    # This case seems to happen if you uninstall AVR Studio 5.0 after installing AVR Studio 5.1.
    # Another bad thing that happens is that the Visual Assist X for AVR Studio extension goes away.
    MessageBox MB_OK "Your AVR Studio 5.1 installation appears to be corrupted.  You can still install the Pololu AVR C/C++ library into AVR Studio 5.1 but the installation of the project templates will probably not work.  You should be able to fix this problem by reinstalling or repairing AVR Studio 5.1 and then running this installer again."

    AS51Done:
FunctionEnd

Function AS60Check
    ReadRegStr $0 HKLM "SOFTWARE\Microsoft\AppEnv\10.0\Apps\AtmelStudio_6.0" 'StubExePath'
    ${GetParent} $0 $AS60Path
    StrCmp $AS60Path "" 0 AS60Done    # If we found it, then we are done.

    ReadRegStr $AS60Path HKLM "SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\{51CC3953-2D06-47FA-832A-B7FD24D01322}" 'InstallLocation'
    StrCmp $AS60Path "" AS60Done      # If Atmel Studio 6.0 really isn't there, we are done.

    # I'm not sure if this case can happen, but it could for AVR Studio 5.1 if
    # you uninstalled AVR Studio 5.0.
    MessageBox MB_OK "Your Atmel Studio 6.0 installation appears to be corrupted.  You can still install the Pololu AVR C/C++ library into Atmel Studio 6.0 but the installation of the project templates will probably not work.  You should be able to fix this problem by reinstalling or repairing Atmel Studio 6.0 and then running this installer again."
    AS60Done:
FunctionEnd

# Typically, AS61Path is:
#   C:\Program Files (x86)\Atmel\Atmel Studio 6.1
# Typically, AS61PathAvr is:
#   C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.2.939\avr8-gnu-toolchain\avr
Function AS61Check
    ReadRegStr $0 HKLM "SOFTWARE\Microsoft\AppEnv\10.0\Apps\AtmelStudio_6.1" 'StubExePath'
    ${GetParent} $0 $AS61Path
    StrCmp $AS61Path "" AS61Done    # If we cannot find it, then we are done.

    # Knowing the path of Atmel Studio 6.1 is not good enough.  We need to find the 8-bit toolchain.
    ReadRegStr $0 HKLM "SOFTWARE\Atmel\AVR8 GCC\6.1\Native" 'InstallDir'
    StrCmp $0 "" AS61Done    # If we can't find this key, give up.
    ReadRegStr $1 HKLM "SOFTWARE\Atmel\AVR8 GCC\6.1\Native" 'Version'
    StrCmp $1 "" AS61Done    # If we can't find this key, give up.
    StrCpy $AS61PathAvr "$0Atmel Toolchain\AVR8 GCC\Native\$1\avr8-gnu-toolchain\avr"

    AS61Done:
FunctionEnd

# Typically, AS62Path is:
#   C:\Program Files (x86)\Atmel\Atmel Studio 6.2
# Typically, AS62PathAvr is:
#   C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.2.939\avr8-gnu-toolchain\avr
Function AS62Check
    ReadRegStr $0 HKLM "SOFTWARE\Microsoft\AppEnv\10.0\Apps\AtmelStudio_6.2" 'StubExePath'
    ${GetParent} $0 $AS62Path
    StrCmp $AS62Path "" AS62Done    # If we cannot find it, then we are done.

    # Knowing the path of Atmel Studio 6.2 is not good enough.  We need to find the 8-bit toolchain.
    ReadRegStr $0 HKLM "SOFTWARE\Atmel\AVR8 GCC\6.2\Native" 'InstallDir'
    StrCmp $0 "" AS62Done    # If we can't find this key, give up.
    ReadRegStr $1 HKLM "SOFTWARE\Atmel\AVR8 GCC\6.2\Native" 'Version'
    StrCmp $1 "" AS62Done    # If we can't find this key, give up.
    StrCpy $AS62PathAvr "$0Atmel Toolchain\AVR8 GCC\Native\$1\avr8-gnu-toolchain\avr"

    AS62Done:
FunctionEnd

# We need to find the location of the Visual Studio 2010 isolated shell so we can install our extension
# using vsixinstaller.exe.  We check several places in the registry and take the first one.
# The registry keys suggested by Microsoft are better:
#  http://osherove.com/blog/2010/4/9/finding-the-location-of-vsixinstallerexe-programmatically.html
# This one might be even better because it doesn't contain the language but we are not using it yet:
#    HKLM "SOFTWARE\Microsoft\VisualStudio\10.0\" 'InstallDir'
#  http://msdn.microsoft.com/en-us/library/bb932484.aspx
Function VSShellCheck
    ReadRegStr $VSShellPath HKLM "Software\Microsoft\VisualStudio\10.0\Setup\IsoShell\1033" 'ProductDir'
    StrCmp $VSShellPath "" 0 VSShellCheckDone
    ReadRegStr $VSShellPath HKCU "SOFTWARE\Atmel\AvrStudio\5.0_Config" 'ShellFolder'
    StrCmp $VSShellPath "" 0 VSShellCheckDone
    ReadRegStr $VSShellPath HKCU "Software\Atmel\AVRStudio51\5.1_Config" 'ShellFolder'
    StrCmp $VSShellPath "" 0 VSShellCheckDone
    ReadRegStr $VSShellPath HKCU "Software\Atmel\AtmelStudio\6.0_Config" 'ShellFolder'
    StrCmp $VSShellPath "" 0 VSShellCheckDone
    ReadRegStr $VSShellPath HKCU "Software\Atmel\AtmelStudio\6.1_Config" 'ShellFolder'
    # Note: Software\Atmel\AtmelStudio\6.2_Config does not exist
    VSShellCheckDone:
FunctionEnd

Function checkDirectory
    # Warn the user if they choose an existing directory.
    ${if} ${FileExists} $INSTDIR
        MessageBox MB_OKCANCEL|MB_ICONEXCLAMATION "The folder $INSTDIR already exists.  Installing to this directory could overwrite existing files." /SD IDOK IDOK dirOk
        Abort
        dirOk:
    ${endif}
FunctionEnd