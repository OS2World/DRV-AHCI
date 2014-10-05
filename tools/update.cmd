/* Update the driver */
call RxFuncAdd 'SysLoadFuncs', 'RexxUtil', 'SysLoadFuncs';
call SysLoadFuncs;
parse arg BootDrive;

if (BootDrive='') then BootDrive=SysBootDrive();

address CMD 'copy /b OS2AHCI.ADD '||BootDrive||'\OS2\BOOT';
address CMD 'copy /b OS2AHCI.SYM '||BootDrive||'\OS2\BOOT';

