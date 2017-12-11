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

        protected override void OnCreate(Bundle bundle)
        {
            base.OnCreate(bundle);

            // Set our view from the "main" layout resource
            SetContentView(Resource.Layout.MainSettings);

            var edtAddress = FindViewById<EditText>(Resource.Id.edtAddress);
            var edtPort = FindViewById<EditText>(Resource.Id.edtPort);
            //var edtEmail = FindViewById<EditText>(Resource.Id.edtEmail);
            var btnEdit = FindViewById<Button>(Resource.Id.btnEdit);

            btnEdit.Click += delegate
            {
                settings.EditUrl(edtPort.Text, edtAddress.Text);
            };
        }
    }
}