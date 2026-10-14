Add-Type -AssemblyName System.Runtime.WindowsRuntime
[Windows.Networking.Connectivity.NetworkInformation, Windows.Networking.Connectivity, ContentType = WindowsRuntime] | Out-Null
[Windows.Networking.NetworkOperators.NetworkOperatorTetheringManager, Windows.Networking.NetworkOperators, ContentType = WindowsRuntime] | Out-Null

$profile = [Windows.Networking.Connectivity.NetworkInformation]::GetInternetConnectionProfile()
if ($profile) {
    $manager = [Windows.Networking.NetworkOperators.NetworkOperatorTetheringManager]::CreateFromConnectionProfile($profile)
    Write-Output "Hotspot State: $($manager.TetheringOperationalState)"
    Write-Output "Client Count: $($manager.ClientCount)"
    $clients = $manager.GetTetheringClients()
    foreach ($c in $clients) {
        Write-Output "Client: MAC=$($c.MacAddress) Host=$($c.HostName)"
    }
}
