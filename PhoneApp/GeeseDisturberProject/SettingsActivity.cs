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
using GeeseDisturberProject.Resources.DataHelper;
using GeeseDisturberProject.Model;
using GeeseDisturberProject.Resources;
using Android.Util;

namespace GeeseDisturberProject.Settings
{
    [Activity(Label = "Main Settings")]
    public class SettingsActivity : Activity
    {
        Setting settings = new Setting();
        bool save = false;
        //ListView lstData;
        //List<History> lstSource = new List<History>();
        //DataBase db;
        protected override void OnCreate(Bundle bundle)
        {
            base.OnCreate(bundle);

            // Set our view from the "main" layout resource
            SetContentView(Resource.Layout.MainSettings);

            var edtAddress = FindViewById<EditText>(Resource.Id.edtAddress);
            var edtPort = FindViewById<EditText>(Resource.Id.edtPort);
            edtPort.Hint = settings.Port_n;

            var btnEdit = FindViewById<Button>(Resource.Id.btnEdit);

            btnEdit.Click += (e, o) =>
            {
                settings.EditUrl(edtPort.Text, edtAddress.Text);
                Toast.MakeText(this, "Saved", ToastLength.Long).Show();
            };
                

            //btnEdit.Click += delegate
            // {
            //     settings.EditUrl(edtPort.Text, edtAddress.Text);
            // };
        }
    }
}