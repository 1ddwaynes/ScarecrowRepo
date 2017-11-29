using Android.App;
using Android.OS;
using Android.Webkit;
using Android.Widget;
using GeeseDisturberProject.Resources.DataHelper;
using System;
using static GeeseDisturberProject.Camera.CameraActivity;

namespace GeeseDisturberProject
{
    [Activity(Label = "ControlPadActivity")]
    public class ControlPadActivity : Activity
    {
        WebView web_view;
        DataBase db;
        string folder = System.Environment.GetFolderPath(System.Environment.SpecialFolder.Personal);

        private Button btnUp;
        private Button btnDown;
        private Button btnLeft;
        private Button btnRight;

    
        static string url = "http://proxy7.remote-iot.com:";
        static string port = "29444";

        static string stream = url + port + "/stream";
        static string StreamSetting = url + port + "/panel";

        protected override void OnCreate(Bundle savedInstanceState)
        {
            base.OnPostCreate(savedInstanceState);

            // Set our view from the "main" layout resource
            SetContentView(Resource.Layout.ControlsPadActivity);
            CameraView();
            FindViews();
            HandleEvents();
        }

        public void CameraView()
        {
            web_view = FindViewById<WebView>(Resource.Id.webView1);
            web_view.Settings.JavaScriptEnabled = true;
            web_view.SetWebViewClient(new HelloWebViewClient());

            int default_zoom_level = 100;
            web_view.SetInitialScale(default_zoom_level);

            var width = web_view.Width;
            var height = web_view.Height;

            SetViewSettings(web_view);

            web_view.LoadUrl(stream + "?width=" + width + "&height=" + height);
        }

        private void SetViewSettings(WebView view)
        {
            view.Settings.LoadWithOverviewMode = true;
            view.Settings.UseWideViewPort = true;
        }

        private void FindViews()
        {
            btnUp = FindViewById<Button>(Resource.Id.btnUp);
            btnDown = FindViewById<Button>(Resource.Id.btnDown);
            btnLeft = FindViewById<Button>(Resource.Id.btnLeft);
            btnRight = FindViewById<Button>(Resource.Id.btnRight);
        }

        private void HandleEvents()
        {
            btnUp.Click += btnUp_Click;
            btnDown.Click += btnDown_Click;
            btnLeft.Click += btnLeft_Click;
            btnRight.Click += btnRight_Click;
        }

        private void btnRight_Click(object sender, EventArgs e)
        {
            throw new NotImplementedException();
        }

        private void btnLeft_Click(object sender, EventArgs e)
        {
            throw new NotImplementedException();
        }

        private void btnDown_Click(object sender, EventArgs e)
        {
            throw new NotImplementedException();
        }

        private void btnUp_Click(object sender, EventArgs e)
        {
            throw new NotImplementedException();
        }
    }
}