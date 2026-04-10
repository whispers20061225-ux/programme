param(
    [string]$PortName = "COM4",
    [int]$BaudRate = 115200,
    [string]$ListenHost = "127.0.0.1",
    [int]$ListenPort = 19024,
    [int]$ReadTimeoutMs = 1000
)

$serial = [System.IO.Ports.SerialPort]::new(
    $PortName,
    $BaudRate,
    [System.IO.Ports.Parity]::None,
    8,
    [System.IO.Ports.StopBits]::One
)
$serial.NewLine = "`n"
$serial.ReadTimeout = $ReadTimeoutMs
$serial.WriteTimeout = $ReadTimeoutMs

$listener = $null

try {
    $serial.Open()
    Write-Host "stm32 tcp bridge: serial open on $PortName @ $BaudRate"

    $listenAddress = [System.Net.IPAddress]::Parse($ListenHost)
    $listener = [System.Net.Sockets.TcpListener]::new($listenAddress, $ListenPort)
    $listener.Start()
    Write-Host "stm32 tcp bridge: listening on $ListenHost`:$ListenPort"

    while ($true) {
        $client = $listener.AcceptTcpClient()
        $client.NoDelay = $true
        Write-Host "stm32 tcp bridge: client connected"

        $stream = $null
        $reader = $null
        $writer = $null

        try {
            $stream = $client.GetStream()
            $utf8NoBom = [System.Text.UTF8Encoding]::new($false)
            $reader = [System.IO.StreamReader]::new(
                $stream,
                $utf8NoBom,
                $false,
                1024,
                $true
            )
            $writer = [System.IO.StreamWriter]::new(
                $stream,
                $utf8NoBom,
                1024,
                $true
            )
            $writer.NewLine = "`n"
            $writer.AutoFlush = $true

            while ($client.Connected) {
                $request = $reader.ReadLine()
                if ($null -eq $request) {
                    break
                }
                $request = $request.Trim()
                if ([string]::IsNullOrWhiteSpace($request)) {
                    continue
                }

                try {
                    $serial.DiscardInBuffer()
                } catch {
                }

                $serial.WriteLine($request)

                $response = ""
                try {
                    $response = $serial.ReadLine()
                } catch [System.TimeoutException] {
                    $response = ""
                }

                if ($null -eq $response) {
                    $response = ""
                }

                $writer.WriteLine($response.Trim())
            }
        } catch {
            Write-Warning ("stm32 tcp bridge: client session failed: " + $_.Exception.Message)
        } finally {
            if ($writer -ne $null) {
                $writer.Dispose()
            }
            if ($reader -ne $null) {
                $reader.Dispose()
            }
            if ($stream -ne $null) {
                $stream.Dispose()
            }
            $client.Close()
            Write-Host "stm32 tcp bridge: client disconnected"
        }
    }
} finally {
    if ($listener -ne $null) {
        $listener.Stop()
    }
    if ($serial.IsOpen) {
        $serial.Close()
    }
}
