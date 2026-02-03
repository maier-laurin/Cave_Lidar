using System;
using System.Collections.Generic;
using System.IO;
using Apache.Arrow;
using Apache.Arrow.Ipc;
using Apache.Arrow.Types;
using Intel.RealSense;

class Program
{
    static void Main(string[] args)
    {
        string fileName = "t265_full_track.arrow";
        int bufferSize = 500; // Sized for ~2.5 seconds of data at 200Hz

        //* Initialize all lists with size hints
        var ts = new List<double>(bufferSize);
        var t_x = new List<float>(bufferSize); var t_y = new List<float>(bufferSize); var t_z = new List<float>(bufferSize);
        var r_x = new List<float>(bufferSize); var r_y = new List<float>(bufferSize); var r_z = new List<float>(bufferSize); var r_w = new List<float>(bufferSize);
        var v_x = new List<float>(bufferSize); var v_y = new List<float>(bufferSize); var v_z = new List<float>(bufferSize);
        var a_x = new List<float>(bufferSize); var a_y = new List<float>(bufferSize); var a_z = new List<float>(bufferSize);
        var av_x = new List<float>(bufferSize); var av_y = new List<float>(bufferSize); var av_z = new List<float>(bufferSize);
        var aa_x = new List<float>(bufferSize); var aa_y = new List<float>(bufferSize); var aa_z = new List<float>(bufferSize);
        var conf = new List<byte>(bufferSize);

        //* build data schema
        var schemaBuilder = new Schema.Builder()
            .Field(f => f.Name("timestamp").DataType(DoubleType.Default).Nullable(false))
            //§ Translation
            .Field(f => f.Name("t_x").DataType(FloatType.Default).Nullable(false))
            .Field(f => f.Name("t_y").DataType(FloatType.Default).Nullable(false))
            .Field(f => f.Name("t_z").DataType(FloatType.Default).Nullable(false))
            //§ Rotation (Quaternions)
            .Field(f => f.Name("r_x").DataType(FloatType.Default).Nullable(false))
            .Field(f => f.Name("r_y").DataType(FloatType.Default).Nullable(false))
            .Field(f => f.Name("r_z").DataType(FloatType.Default).Nullable(false))
            .Field(f => f.Name("r_w").DataType(FloatType.Default).Nullable(false))
            //§ Velocity
            .Field(f => f.Name("v_x").DataType(FloatType.Default).Nullable(false))
            .Field(f => f.Name("v_y").DataType(FloatType.Default).Nullable(false))
            .Field(f => f.Name("v_z").DataType(FloatType.Default).Nullable(false))
            //§ Acceleration
            .Field(f => f.Name("a_x").DataType(FloatType.Default).Nullable(false))
            .Field(f => f.Name("a_y").DataType(FloatType.Default).Nullable(false))
            .Field(f => f.Name("a_z").DataType(FloatType.Default).Nullable(false))
            //§ Angular Velocity
            .Field(f => f.Name("av_x").DataType(FloatType.Default).Nullable(false))
            .Field(f => f.Name("av_y").DataType(FloatType.Default).Nullable(false))
            .Field(f => f.Name("av_z").DataType(FloatType.Default).Nullable(false))
            //§ Angular Acceleration
            .Field(f => f.Name("aa_x").DataType(FloatType.Default).Nullable(false))
            .Field(f => f.Name("aa_y").DataType(FloatType.Default).Nullable(false))
            .Field(f => f.Name("aa_z").DataType(FloatType.Default).Nullable(false))
            //§ Confidence
            .Field(f => f.Name("confidence").DataType(UInt8Type.Default).Nullable(false));

        var schema = schemaBuilder.Build();

        using var pipe = new Pipeline(); // for the InteliSense connection
        var cfg = new Config();
        cfg.EnableStream(Intel.RealSense.Stream.Pose); // Pose includes all the Slam Data, but not the fischeye kammera images, and the raw sensor data
        pipe.Start(cfg);

        Console.WriteLine($"Recording... Writing to {fileName}. Press Ctrl+C to exit.");

        using (var stream = File.Create(fileName))
        using (var writer = new ArrowFileWriter(stream, schema))
        {
            try
            {
                while (true)
                {
                    using (var frames = pipe.WaitForFrames())
                    using (var poseFrame = frames.FirstOrDefault<PoseFrame>(Intel.RealSense.Stream.Pose))
                    {
                        if (poseFrame != null)
                        {
                            var d = poseFrame.PoseData;

                            ts.Add(poseFrame.Timestamp);

                            t_x.Add(d.translation.x); 
                            t_y.Add(d.translation.y); 
                            t_z.Add(d.translation.z);

                            r_x.Add(d.rotation.x); 
                            r_y.Add(d.rotation.y); 
                            r_z.Add(d.rotation.z); 
                            r_w.Add(d.rotation.w);

                            v_x.Add(d.velocity.x); 
                            v_y.Add(d.velocity.y); 
                            v_z.Add(d.velocity.z);

                            a_x.Add(d.acceleration.x); 
                            a_y.Add(d.acceleration.y); 
                            a_z.Add(d.acceleration.z);

                            av_x.Add(d.angular_velocity.x); 
                            av_y.Add(d.angular_velocity.y); 
                            av_z.Add(d.angular_velocity.z);

                            aa_x.Add(d.angular_acceleration.x); 
                            aa_y.Add(d.angular_acceleration.y); 
                            aa_z.Add(d.angular_acceleration.z);
                            
                            conf.Add((byte)d.tracker_confidence);

                            if (ts.Count >= bufferSize)
                            {
                                // Create Batch
                                var batch = new RecordBatch(schema, new IArrowArray[] 
                                {
                                    new DoubleArray.Builder().AppendRange(ts).Build(),
                                    new FloatArray.Builder().AppendRange(t_x).Build(),
                                    new FloatArray.Builder().AppendRange(t_y).Build(),
                                    new FloatArray.Builder().AppendRange(t_z).Build(),
                                    new FloatArray.Builder().AppendRange(r_x).Build(),
                                    new FloatArray.Builder().AppendRange(r_y).Build(),
                                    new FloatArray.Builder().AppendRange(r_z).Build(),
                                    new FloatArray.Builder().AppendRange(r_w).Build(),
                                    new FloatArray.Builder().AppendRange(v_x).Build(),
                                    new FloatArray.Builder().AppendRange(v_y).Build(),
                                    new FloatArray.Builder().AppendRange(v_z).Build(),
                                    new FloatArray.Builder().AppendRange(a_x).Build(),
                                    new FloatArray.Builder().AppendRange(a_y).Build(),
                                    new FloatArray.Builder().AppendRange(a_z).Build(),
                                    new FloatArray.Builder().AppendRange(av_x).Build(),
                                    new FloatArray.Builder().AppendRange(av_y).Build(),
                                    new FloatArray.Builder().AppendRange(av_z).Build(),
                                    new FloatArray.Builder().AppendRange(aa_x).Build(),
                                    new FloatArray.Builder().AppendRange(aa_y).Build(),
                                    new FloatArray.Builder().AppendRange(aa_z).Build(),
                                    new UInt8Array.Builder().AppendRange(conf).Build()
                                }, ts.Count);

                                writer.WriteRecordBatch(batch);
                                
                                // Reset lists (keeps capacity)
                                ts.Clear(); t_x.Clear(); t_y.Clear(); t_z.Clear();
                                r_x.Clear(); r_y.Clear(); r_z.Clear(); r_w.Clear();
                                v_x.Clear(); v_y.Clear(); v_z.Clear();
                                a_x.Clear(); a_y.Clear(); a_z.Clear();
                                av_x.Clear(); av_y.Clear(); av_z.Clear();
                                aa_x.Clear(); aa_y.Clear(); aa_z.Clear();
                                conf.Clear();

                                Console.Write(">"); 
                            }
                        }
                    }
                }
            }
            catch (OperationCanceledException) { /* Handle exit */ }
            finally
            {
                writer.WriteEnd();
                pipe.Stop();
            }
        }
    }
}