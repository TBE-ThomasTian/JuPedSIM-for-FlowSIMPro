; Inno Setup script for the FlowSIM Pro Evac Add-On (the JuPedSim solver CLI).
;
; This is the Windows counterpart to the CPack packaging in CMakeLists.txt: it
; produces the same install tree under the same prefix, so FlowSIM Pro finds the
; solver either way. Unlike the CPack/NSIS path it can be signed with the
; GlobalSign token.
;
;   ISCC.exe scripts\installer\flowsimpro-evac-addon.iss
;
; Overridable with ISCC /D<name>=<value>:
;   BinDir        directory holding the built jupedsim.exe
;   MyAppVersion  release number stamped into the installer (default 2026)
;   OutDir        where the installer is written (default build\installer)
;   SkipSign      define it to build unsigned, e.g. on a machine without the
;                 token: ISCC /DSkipSign ...

#define MyAppName "FlowSIM Pro Evac Add-On"
#define MyAppDirName "FlowSIMProEvacAddOn"
#define MyAppPublisher "FlowSIM Pro"
#define MyAppURL "https://www.flowsimpro.com/"
#define MyAppExeName "jupedsim.exe"

; Keep in sync with FLOWSIMPRO_ADDON_VERSION in CMakeLists.txt. Year-based, and
; deliberately not the upstream JuPedSim version from project(JuPedSim VERSION).
; MAJOR.MINOR.PATCH, so a corrected build of the same year can be told apart in
; Apps & Features - with a bare year it cannot.
#ifndef MyAppVersion
  #define MyAppVersion "2026.1.0"
#endif

#define RepoRoot AddBackslash(SourcePath) + "..\..\"

#ifndef OutDir
  #define OutDir RepoRoot + "build\installer"
#endif

; Default to the Qt Creator build tree, fall back to the layout the README
; documents for the Ninja build.
#ifndef BinDir
  #define BinDir RepoRoot + "build\Desktop_Qt_6_11_1_MSVC2022_64bit_Release\bin"
  #if !FileExists(BinDir + "\" + MyAppExeName)
    #undef BinDir
    #define BinDir RepoRoot + "build\bin"
  #endif
#endif

#if !FileExists(BinDir + "\" + MyAppExeName)
  #error Could not find jupedsim.exe. Build it first, or pass /DBinDir=<path to the bin directory>.
#endif

[Setup]
AppId={{9C9182B8-EA8E-4514-B3AC-A81E06BCA9F8}
AppName={#MyAppName}
AppVerName={#MyAppName} {#MyAppVersion}
AppVersion={#MyAppVersion}

; Inno does not derive VersionInfoVersion from AppVersion. Without it the setup
; EXE carries an empty FileVersion, which is the field software distribution,
; inventory tools and winget read - AppVersion alone only fills ProductVersion.
VersionInfoVersion={#MyAppVersion}
VersionInfoCompany={#MyAppPublisher}
VersionInfoDescription={#MyAppName} Setup

AppPublisher={#MyAppPublisher}
AppPublisherURL={#MyAppURL}
AppSupportURL={#MyAppURL}
AppUpdatesURL={#MyAppURL}

; --- Update-Verhalten ---
; The AppId above is what ties an update to the existing installation: same
; AppId means Inno Setup upgrades in place instead of installing a second copy.
DefaultDirName={autopf}\{#MyAppDirName}
UsePreviousAppDir=yes
DefaultGroupName={#MyAppName}
UsePreviousGroup=yes

; --- Installer UX ---
AllowNoIcons=yes
WizardStyle=modern
; ShowLanguageDialog=auto

; --- Build / Output ---
OutputDir={#OutDir}
; Same name the NSIS generator produces, so this is a drop-in replacement.
OutputBaseFilename=flowsimpro-evac-addon-{#MyAppVersion}-win64
Compression=lzma
SolidCompression=yes

; --- Rechte / Architektur ---
; admin without PrivilegesRequiredOverridesAllowed, on purpose: FlowSIM Pro
; probes the fixed path C:\Program Files\FlowSIMProEvacAddOn\bin\jupedsim.exe
; (JuPedSimHelper::defaultCandidates()). A non-admin install would land in
; {localappdata}\Programs and auto-detection would silently miss it.
PrivilegesRequired=admin
; Also required for the fixed path: without it {autopf} is Program Files (x86).
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible

; --- Signierung ---
#ifndef SkipSign
SignTool=GlobalSignToken
SignedUninstaller=yes
#endif

; --- Icon ---
; The FlowSIM Pro product icon with the evacuation badge that tells the add-on
; apart from the main product, built by scripts\installer\make_icon.ps1 at
; 16..256 px. Deliberately not the JuPedSim mark: the LGPL covers the upstream
; code, not the upstream project's logo, and a signed installer carrying it
; would read as an official release from Forschungszentrum Jülich.
; jupedsim.exe is a console program and carries no icon of its own, so the entry
; in Apps & Features gets this one instead of a generic shell icon.
SetupIconFile={#RepoRoot}scripts\installer\flowsimpro-evac-addon.ico
UninstallDisplayIcon={app}\flowsimpro-evac-addon.ico

; --- Lizenz ---
LicenseFile={#RepoRoot}LICENSE

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"
Name: "german";  MessagesFile: "compiler:Languages\German.isl"
Name: "korean";  MessagesFile: "compiler:Languages\Korean.isl"
Name: "chinesesimplified"; MessagesFile: "compiler:Languages\ChineseSimplified.isl"
Name: "chinesetraditional"; MessagesFile: "compiler:Languages\ChineseTraditional.isl"

[CustomMessages]
; --- Start menu links ---
english.ExamplesLink=Example scenarios
german.ExamplesLink=Beispielszenarien
korean.ExamplesLink=예제 시나리오
chinesesimplified.ExamplesLink=示例场景
chinesetraditional.ExamplesLink=範例情境

english.WebsiteLink=FlowSIM Pro Website
german.WebsiteLink=FlowSIM Pro Webseite
korean.WebsiteLink=FlowSIM Pro 웹사이트
chinesesimplified.WebsiteLink=FlowSIM Pro 网站
chinesetraditional.WebsiteLink=FlowSIM Pro 網站

; --- Run entry after install ---
english.OpenExamples=Open the example scenarios folder
german.OpenExamples=Ordner mit Beispielszenarien öffnen
korean.OpenExamples=예제 시나리오 폴더 열기
chinesesimplified.OpenExamples=打开示例场景文件夹
chinesetraditional.OpenExamples=開啟範例情境資料夾

; --- Solver busy messages (shown if jupedsim.exe is running) ---
english.SolverBusyTitle=A simulation is still running
german.SolverBusyTitle=Es läuft noch eine Simulation
korean.SolverBusyTitle=시뮬레이션이 아직 실행 중입니다
chinesesimplified.SolverBusyTitle=模拟仍在运行
chinesetraditional.SolverBusyTitle=模擬仍在執行

english.SolverBusyMsg=Please wait for the running evacuation simulation to finish, or close FlowSIM Pro, to continue.
german.SolverBusyMsg=Bitte warten Sie, bis die laufende Entfluchtungssimulation beendet ist, oder schließen Sie FlowSIM Pro, um fortzufahren.
korean.SolverBusyMsg=실행 중인 대피 시뮬레이션이 끝날 때까지 기다리거나 FlowSIM Pro를 종료한 후 계속하세요.
chinesesimplified.SolverBusyMsg=请等待正在运行的疏散模拟完成，或关闭 FlowSIM Pro，然后继续。
chinesetraditional.SolverBusyMsg=請等待正在執行的疏散模擬完成，或關閉 FlowSIM Pro，然後繼續。

english.SolverBusyAbort=Aborted because a simulation is still running.
german.SolverBusyAbort=Abgebrochen, weil noch eine Simulation läuft.
korean.SolverBusyAbort=시뮬레이션이 아직 실행 중이므로 취소되었습니다.
chinesesimplified.SolverBusyAbort=由于模拟仍在运行，已中止。
chinesetraditional.SolverBusyAbort=由於模擬仍在執行，已中止。

[Files]
; signonce, not sign: a binary the build already signed is not signed twice.
Source: "{#BinDir}\{#MyAppExeName}"; DestDir: "{app}\bin"; Flags: ignoreversion signonce

; The layout below matches the install() rules under BUILD_INSTALLER in
; CMakeLists.txt, so this installer and the CPack packages produce the same tree.
; Filtered by extension rather than copied wholesale, so the .jsp files a local
; example run leaves behind do not end up in the package.
Source: "{#RepoRoot}examples\xml\*.xml"; DestDir: "{app}\share\jupedsim\examples"; Flags: ignoreversion
Source: "{#RepoRoot}examples\xml\*.md";  DestDir: "{app}\share\jupedsim\examples"; Flags: ignoreversion
Source: "{#RepoRoot}LICENSE";            DestDir: "{app}\share\jupedsim"; Flags: ignoreversion
Source: "{#RepoRoot}README.md";          DestDir: "{app}\share\jupedsim"; Flags: ignoreversion

; LGPLv3 obliges whoever conveys the binary to make the corresponding source
; available. This names where it is; without it the installed product carries a
; licence it gives no way to act on.
Source: "{#RepoRoot}SOURCE.txt";         DestDir: "{app}\share\jupedsim"; Flags: ignoreversion

; Only file in the tree that CPack does not also install: UninstallDisplayIcon
; needs an icon that still exists after Setup has finished.
Source: "{#RepoRoot}scripts\installer\flowsimpro-evac-addon.ico"; DestDir: "{app}"; Flags: ignoreversion

[Icons]
; No shortcut to the solver itself: jupedsim.exe is a command line program that
; FlowSIM Pro invokes, double-clicking it would only flash a console window.
Name: "{group}\{cm:ExamplesLink}"; Filename: "{app}\share\jupedsim\examples"
Name: "{group}\{cm:WebsiteLink}";  Filename: "{#MyAppURL}"

[Run]
Filename: "{win}\explorer.exe"; \
    Parameters: """{app}\share\jupedsim\examples"""; \
    Description: "{cm:OpenExamples}"; \
    Flags: nowait postinstall skipifsilent

[UninstallDelete]
; Running an example in place writes a .jsp next to the scenario, which would
; otherwise keep {app} from being removed.
Type: filesandordirs; Name: "{app}\share\jupedsim\examples"
Type: dirifempty;     Name: "{app}\share\jupedsim"
Type: dirifempty;     Name: "{app}\share"
Type: dirifempty;     Name: "{app}\bin"
Type: dirifempty;     Name: "{app}"

[Code]
function IsSolverRunning: Boolean;
var
  ResultCode: Integer;
begin
  { tasklist ist auf Windows verfügbar; wir suchen genau unseren Prozessnamen }
  Result := Exec(ExpandConstant('{cmd}'), '/C tasklist /FI "IMAGENAME eq {#MyAppExeName}" | find /I "{#MyAppExeName}" >nul',
                 '', SW_HIDE, ewWaitUntilTerminated, ResultCode) and (ResultCode = 0);
end;

{ Bis zu 12x nachfragen, dann aufgeben. Eine laufende Simulation hält
  jupedsim.exe offen, das Überschreiben würde sonst mitten im Setup scheitern. }
function WaitForSolverToStop: Boolean;
var
  Retry: Integer;
begin
  Result := True;

  for Retry := 1 to 12 do
  begin
    if not IsSolverRunning then
      exit;

    MsgBox(ExpandConstant('{cm:SolverBusyTitle}') + #13#10#13#10 +
           ExpandConstant('{cm:SolverBusyMsg}'),
           mbInformation, MB_OK);
  end;

  if IsSolverRunning then
  begin
    MsgBox(ExpandConstant('{cm:SolverBusyAbort}'), mbCriticalError, MB_OK);
    Result := False;
  end;
end;

function InitializeSetup(): Boolean;
begin
  Result := WaitForSolverToStop;
end;

function InitializeUninstall(): Boolean;
begin
  Result := WaitForSolverToStop;
end;
