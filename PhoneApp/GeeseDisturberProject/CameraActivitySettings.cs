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
using GeeseDisturberProject.Camera;
using GeeseDisturberProject.Resources.DataHelper;
using GeeseDisturberProject.Model;
using Android.Util;
using Android.Content.PM;

namespace GeeseDisturberProject
{
    [Activity(Label = "CameraActivitySettings", ConfigurationChanges = ConfigChanges.ScreenSize | ConfigChanges.Orientation,
    ScreenOrientation = ScreenOrientation.Landscape)]
    public class CameraActivitySettings : Activity
    {
        WebView web_view;
        private Button BackButton;

        Setting settings = new Setting();

        protected override void OnCreate(Bundle savedInstanceState)
        {
            //var setting = new Setting();
            string data = settings.GetUrl(true);
            Console.WriteLine(settings.GetUrl(true));
            //string data = "http://proxy7.remote-iot.com:13907/panel";


            base.OnCreate(savedInstanceState);

            SetContentView(Resource.Layout.CameraActivitySettings);

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

            var metrics = Resources.DisplayMetrics;
            var width = metrics.WidthPixels;
            var height = metrics.HeightPixels;

            SetViewSettings(web_view);

            web_view.LoadUrl(data + "?width=" + 320 + "&height=" + 240);
        }

        private void HandleEvents()
        {
            BackButton.Click += BackButton_Click;
        }

        private void FindViews()
        {
            BackButton = FindViewById<Button>(Resource.Id.Back);
        }

        private void SetViewSettings(WebView view)
        {
            view.Settings.LoadWithOverviewMode = true;
            view.Settings.UseWideViewPort = true;
        }

        private void BackButton_Click(object sender, EventArgs e)
        {
            var intent = new Intent(this, typeof(CameraActivity));
            Finish();
            StartActivity(intent);
        }

        public class HelloWebViewClient : WebViewClient
        {
            public override bool ShouldOverrideUrlLoading(WebView view, string url)
            {
                view.LoadUrl(url);
                return false;
            }
        }
    }
}