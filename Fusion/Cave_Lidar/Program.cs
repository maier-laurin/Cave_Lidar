using RealSense;
using Velodyne;

class Program
{
    static async Task Main(string[] args)
    {
        var processor = new VelodyneStream();
        var cts = new CancellationTokenSource();

        Console.WriteLine("Press 'S' to stop recording...");

        // Allow Ctrl+C to trigger the stop token
        Console.CancelKeyPress += (s, e) => {
            e.Cancel = true;
            cts.Cancel();
        };

        var producerTask = processor.ProducePackets(cts.Token);
        var consumerTask = processor.ConsumeAndWrite("vlp16_data.arrow", 1500, cts.Token);
        var t265Recorder = new T265ArrowRecorder("t265_data.arrow", 1500);
        var t265Task = t265Recorder.RunAsync(cts.Token);

        Console.WriteLine("Starting capture threads...");
        

        while (true)
        {
            if (Console.KeyAvailable && Console.ReadKey(true).Key == ConsoleKey.S)
            {
                cts.Cancel();
                break;
            }
            await Task.Delay(100);
        }


        await Task.WhenAll(t265Task, producerTask, consumerTask);

        Console.WriteLine("All systems shut down cleanly.");
    }
}