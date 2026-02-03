
using VelodyneCapture;

class Program
{
    static async Task Main(string[] args)
    {
        var processor = new VelodyneStream();
        var cts = new CancellationTokenSource();

        Console.WriteLine("Press 'S' to stop recording...");

        var producerTask = processor.ProducePackets(cts.Token);
        var consumerTask = processor.ConsumeAndWrite("output.arrow", 1500, cts.Token);

        // Simple loop to wait for user to stop
        while (true)
        {
            if (Console.KeyAvailable && Console.ReadKey(true).Key == ConsoleKey.S)
            {
                cts.Cancel();
                break;
            }
            await Task.Delay(100);
        }

        await Task.WhenAll(producerTask, consumerTask);
        Console.WriteLine("Capture finished safely.");
    }
}