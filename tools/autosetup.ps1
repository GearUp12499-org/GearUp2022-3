function get_git {
    Write-Output "Trying to find git..."
    $gitfound = Get-Command "git.exe" -ErrorAction SilentlyContinue
    if ($gitfound) {
        Write-Output "git ok"
    } else {
        Write-Output "Didn't find git. Trying to locate Git for Windows on the system"
        # Try to find Git for Windows on the system
        # Check common installation locations
        $testloc = "C:\Program Files\Git\cmd\git.exe"
        if (Test-Path $testloc) {
            Write-Output "Found Git for Windows at $testloc"
            $gitpath = $testloc
        } else {
            $testloc = "C:\Program Files (x86)\Git\cmd\git.exe"
            if (Test-Path $testloc) {
                Write-Output "Found Git for Windows at $testloc"
                $gitpath = $testloc
            } else {
                Write-Output "Didn't find Git for Windows on the system, sorry!";
                $gitpath = ""
                return
            }
        }

        Write-Output "Updating PATH to include $gitpath"
        [System.Environment]::SetEnvironmentVariable("PATH", "$env:PATH;$gitpath", "User")
        Write-Output "New path: $env:path"
    }

    Write-Output "Checking git config 1/2"
    $usernameresult = (git.exe config --global user.name) | Out-String
    $usernameresult = $usernameresult -replace "\s"
    
    Write-Output "Checking git config 2/2"
    $useremailresult = (git.exe config --global user.email) | Out-String
    $useremailresult = $useremailresult -replace "\s"
    if ($usernameresult -eq "") {
        Write-Output "No name set. What is your name?"
        $newname = Read-Host -Prompt "Username: "
        git config --global user.name "$newname"
    } else {
        Write-Output "Name OK"
    }
    if ($useremailresult -eq "") {
        Write-Output "No email set. What is your email?"
        Write-Output "If using GitHub, take your email from the GitHub Settings > Email page"
        Write-Output "    to avoid issues while pushing changes."
        $newemail = Read-Host -Prompt "Email: "
        git config --global user.email "$newemail"
    } else {
        Write-Output "Email OK"
    }
}


function get_adb {
    Write-Output "Trying to find adb..."
    $gitfound = Get-Command "adb.exe" -ErrorAction SilentlyContinue
    if ($gitfound) {
        Write-Output "adb ok"
    } else {
        Write-Output "Didn't find adb. Trying to locate ADB on the system"
        # Try to find Git for Windows on the system
        # Check common installation locations
        $testloc = "$env:localappdata\Android\Sdk\platform-tools\adb.exe"
        if (Test-Path $testloc) {
            Write-Output "Found ADB at $testloc"
            $gitpath = $testloc
        } else {
            Write-Output "Didn't find ADB on the system, sorry!";
            return
        }

        Write-Output "Updating PATH to include $gitpath"
        [System.Environment]::SetEnvironmentVariable("PATH", "$env:PATH;$gitpath", "User")
        Write-Output "New path: $env:path"
    }
}


get_git
get_adb
