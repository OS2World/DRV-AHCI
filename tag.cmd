@echo off
setlocal
SET TRUNK_URL=https://svn.ecomstation.nl/repos/ahci/trunk
SET TAGS_URL=https://svn.ecomstation.nl/repos/ahci/tags 

if %1empty == empty goto usage

echo tagging %1...
svn copy %TRUNK_URL% %TAGS_URL%/%1 -m "tagging %1"
goto end

:usage
echo.
echo %0 - create tag in OS2AHCI subversion repository.
echo.
echo Usage: %0 [tag]
echo where [tag] usually is the release version number (e.g. 1.01)"


:end
endlocal
