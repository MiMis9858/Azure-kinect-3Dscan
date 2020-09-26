using System.Collections;
using System.Collections.Generic;
using UnityEngine;
//AzureKinectSDKの読み込み
using Microsoft.Azure.Kinect.Sensor;
//非同期処理をする準備
using System.Threading.Tasks;
using System;
using System.Security.Cryptography;

public class KinectScript : MonoBehaviour
{
    //Kinectを扱う変数
    Device kinect;
    //表示するPointCloudの全点数
    int num;
    //点の集合を描画するために使用(表示する図形の詳細を管理するオブジェクト)
    Mesh mesh;
    //PointCloudの各点の座標の配列
    Vector3[] vertices;

    Vector3 colect_point_cloud = List<Vector3>();

    //PointCloudの各点に対応する色の配列


    Color32[] colors;
    //vertices中の何番目の点を描画するかのリスト(全部描画するけど手続き上必要)
    int[] indices;
    //座標変換(Color⇔Depth対応やDepth→xyzなど)をするためのクラス
    Transformation transformation;
    // キャプチャフラグ（GUI等で起動）
    public static bool captured = false;

    System.Diagnostics.Stopwatch sw; 

    void Start()
    {
        sw = new System.Diagnostics.Stopwatch();
        captured = false;
        //最初の一回だけKinect初期化メソッドを呼び出す
        InitKinect();
        //点群描画のための初期化
        InitMesh();
        //Kinectからのデータ取得と描画(ループ)
        Task t = KinectLoop();
    }

    //Kinectの初期化
    private void InitKinect()
    {
        //0番目のKinectと接続
        kinect = Device.Open(0);
        //Kinectの各種モードを設定して動作開始
        kinect.StartCameras(new DeviceConfiguration
        {
            ColorFormat = ImageFormat.ColorBGRA32,
            ColorResolution = ColorResolution.R720p,
            DepthMode = DepthMode.NFOV_2x2Binned,
            SynchronizedImagesOnly = true,
            CameraFPS = FPS.FPS5 //FPS5, FPS30
        }) ;
        //座標変換(Color⇔Depth対応やDepth→xyz)のための情報を生成
        transformation = kinect.GetCalibration().CreateTransformation();
        sw.Start();
    }

    //Meshを用いてPointCloudを描画する準備をする
    private void InitMesh()
    {
        //Depth画像の横幅と縦幅を取得し、全点数を算出
        int width = kinect.GetCalibration().DepthCameraCalibration.ResolutionWidth;
        int height = kinect.GetCalibration().DepthCameraCalibration.ResolutionHeight;
        num = width * height;
        // サイズ確認
        Debug.Log(width + " " + height);
        //meshをインスタンス化
        mesh = new Mesh();
        //65535点以上を描画するため下記を記述
        mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
        //Depth画像の総ピクセル数分の頂点や色の記憶領域を確保
        vertices = new Vector3[num];
        colors = new Color32[num];
        indices = new int[num];
        //描画する点の配列番号を記録。(全ての点を描画)
        for (int i = 0; i < num; i++)
        {
            indices[i] = i;
        }
        //点の座標や色、描画する点のリストをmeshに渡す
        mesh.vertices = vertices;
        mesh.colors32 = colors;
        mesh.colect_point_cloud = colect_point_cloud;
        mesh.SetIndices(indices, MeshTopology.Points, 0);
        //メッシュをこのスクリプトが貼られているオブジェクトのMashFilterに適用
        gameObject.GetComponent<MeshFilter>().mesh = mesh;
    }

    //Kinectからデータを取得し、描画するメソッド
    private async Task KinectLoop()
    {
        
        int fifteencount = 1;
        //while文でkinectからデータを取り続ける
        while (true)
        {
            //GetCaptureでKinectのデータを取得
            using (Capture capture = await Task.Run(() => kinect.GetCapture()).ConfigureAwait(true))
            {
                //Depth画像との位置・サイズ合わせ済みの画像を取得
                Image colorImage = transformation.ColorImageToDepthCamera(capture);
                //色情報のみの配列を取得
                BGRA[] colorArray = colorImage.GetPixels<BGRA>().ToArray();

                //capture.DepthでDepth画像を取得
                //さらにDepthImageToPointCloudでxyzに変換
                Image xyzImage = transformation.DepthImageToPointCloud(capture.Depth);
                //変換後のデータから点の座標のみの配列を取得
                Short3[] xyzArray = xyzImage.GetPixels<Short3>().ToArray();

                //Kinectで取得した全点の座標や色を代入
                for (int i = 0; i < num; i++)
                {
                    //頂点座標の代入
                    long d = pow3(xyzArray[i]);
                    if (xyzArray[i].Z < 150|| xyzArray[i].Z > 250 || xyzArray[i].Y > 60) xyzArray[i].X = xyzArray[i].Y = xyzArray[i].Z = 0;
                    {
                        vertices[i].x = xyzArray[i].X * 1f;
                        vertices[i].y = -xyzArray[i].Y * 1f;//上下反転　0.001f
                        vertices[i].z = xyzArray[i].Z * 1f;
                        //色の代入
                        /*
                        colors[i].b = colorArray[i].B;
                        colors[i].g = colorArray[i].G;
                        colors[i].r = colorArray[i].R;
                        */
                        colors[i].b = 0;
                        colors[i].g = 0;
                        colors[i].r = 0;
                        colors[i].a = 255;
                    }
                }
                //meshに最新の点の座標と色を渡す
                mesh.vertices = vertices;
                mesh.colors32 = colors;
                mesh.RecalculateBounds();
                // ボタンに対応してキャプチャを行う
                string txt = VertexToText();
                if (captured == true)
                {
                    Save("capture.obj", txt);
                    Debug.Log("Capture & Saved");
                    captured = false;
                }
                // timer 約1秒ごとに処理　±0.1s
                sw.Stop();
                TimeSpan ts = sw.Elapsed;

                int diff = ts.Seconds*1000+ts.Milliseconds - 1000;
                //if (diff < 0) diff = - diff;
                //Debug.Log(ts);
                //Debug.Log(diff);
                if(ts.Seconds *1000 + ts.Milliseconds > 900)
                {
                    ColectScanData();
                    Debug.Log(ts);
                    sw.Restart();
                    
                }
                else
                {
                    sw.Start();
                }
            }
        }
    }
    // メッシュデータをテキスト化
    private string VertexToText()
    {
        // TODO 指定範囲内に限定する
        // TODO ファイルに出力する
        var sb = new System.Text.StringBuilder();
        sb.AppendLine("# positions");
        foreach (var item in mesh.colect_point_cloud)
        {
            // 
            if (item.x == 0 && item.y == 0 && item.z == 0)
                continue;
            sb.AppendFormat("v {0} {1} {2}\n",
                item.x.ToString("F8"),
                item.y.ToString("F8"),
                (item.z-200).ToString("F8"));
        }
        return sb.ToString();
    }

    // ファイルへの保存
    public static bool Save(string path, string objFileText)
    {
        bool ret = false;
        try
        {
            System.IO.File.WriteAllText(path, objFileText);
            ret = true;
        }
        catch (System.Exception e)
        {
            Debug.LogException(e);
        }
        return ret;
    }
    private static void ColectScanData()
    {
        for(int k = 0; k < num ; k++)
        {
            if(mesh.vertices[k].X <= -16 && mesh.vertices[k].X >= -24)
            {
                mesh.vertices[k].X = mesh.vertices[k].x * (float)Math.Cos(24.0*fifteencount) - mesh.vertices[k].z * (float)Math.Sin(24.0*fifteencount);
                mesh.vertices[k].z = mesh.vertices[k].x * (float)Math.Sin(24.0*fifteencount) + mesh.vertices[k].z * (float)Math.Cos(24.0*fifteencount);
                mesh.colect_point_cloud.Add(mesh.vertices[k]);
            }
        }
        fifteencount ++;
    }
    private long pow3(Short3 p)
    {
        return p.X * p.X + p.Y * p.Y + p.Z * p.Z; 
    }
    //このオブジェクトが消える(アプリ終了)と同時にKinectを停止
    private void OnDestroy()
    {
        kinect.StopCameras();
    }

    void Update()
    {

    }
}
