using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using Android.App;
using Android.Content;
using Android.OS;
using Android.Runtime;
using Android.Views;
using Android.Widget;
using Android.Webkit;
using GeeseDisturberProject.Resources;
using GeeseDisturberProject.Resources.DataHelper;
using GeeseDisturberProject.Model;
using Android.Util;

namespace GeeseDisturberProject.Camera
{
    [Activity(Label = "Camera Menu")]
    public class CameraActivity : Activity
    {
        WebView web_view;

        Setting settings = new Setting();
        //DataBase db;
        //string folder = System.Environment.GetFolderPath(System.Environment.SpecialFolder.Personal);

        private Button buttonStreamSetting;
        private Button BackButton;

        protected override void OnCreate(Bundle savedInstanceState)
        {
            base.OnPostCreate(savedInstanceState);
            
            //string data = "http://proxy7.remote-iot.com:12356/stream";
            string data = settings.GetUrl(false);
            Console.WriteLine(settings.GetUrl(false));

            // Set our view from the "main" layout resource
            SetContentView(Resource.Layout.CameraActivity);

            CameraView(data);
            FindViews();
            HandleEvents();
        }

        public void CameraView(string data)
        {
            web_view = FindViewById<WebView>(Resource.Id.WebView);
            web_view.Settings.JavaScriptEnabled = true;
            web_view.SetWebViewClient(new HelloWebViewClient());

            int default_zoom_level = 100;
            web_view.SetInitialScale(default_zoom_level);

            var width = web_view.Width;
            var height = web_view.Height;

            //var metrics = Resources.DisplayMetrics;
            //var width = metrics.WidthPixels;
            //var height = metrics.HeightPixels;

            SetViewSettings(web_view);

            web_view.LoadUrl(data + "?width=" + 320 + "&height=" + 240);
        }

        private void HandleEvents()
        {
            buttonStreamSetting.Click += StreamSettings_Click;
            BackButton.Click += BackButton_Click;
        }

        private void StreamSettings_Click(object sender, EventArgs e)
        {
            var intent = new Intent(this, typeof(CameraActivitySettings));
            
            StartActivity(intent);
        }

        private void BackButton_Click(object sender, EventArgs e)
        {
            var intent = new Intent(this, typeof(MainMenu));
            StartActivity(intent);
        }

        private void FindViews()
        {
            buttonStreamSetting = FindViewById<Button>(Resource.Id.StreamSettings);
            BackButton = FindViewById<Button>(Resource.Id.Back);
        }

        private void SetViewSettings(WebView view)
        {
            view.Settings.LoadWithOverviewMode = true;
            view.Settings.UseWideViewPort = true;
        }

        public class HelloWebViewClient : WebViewClient
        {
            public override bool ShouldOverrideUrlLoading(WebView view, string url)
            {
                view.LoadUrl(url);
                return false;
            }
        }

        public override bool OnKeyDown(Android.Views.Keycode keyCode, Android.Views.KeyEvent e)
        {
            if (keyCode == Keycode.Back && web_view.CanGoBack())
            {
                web_view.GoBack();
                return true;
            }
            return base.OnKeyDown(keyCode, e);
        }
    }
}
