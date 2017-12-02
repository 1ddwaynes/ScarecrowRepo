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
        //ListView lstData;
        //List<History> lstSource = new List<History>();
        //DataBase db;
        protected override void OnCreate(Bundle bundle)
        {
            base.OnCreate(bundle);

            // Set our view from the "main" layout resource
            SetContentView(Resource.Layout.MainSettings);

            //Create DataBase
            //db = new DataBase();
            //db.createDataBase();
            //string folder = System.Environment.GetFolderPath(System.Environment.SpecialFolder.Personal);
            //Log.Info("DB_PATH", folder);

            //lstData = FindViewById<ListView>(Resource.Id.ListView);

            var edtAddress = FindViewById<EditText>(Resource.Id.edtAddress);
            var edtPort = FindViewById<EditText>(Resource.Id.edtPort);
            //var edtEmail = FindViewById<EditText>(Resource.Id.edtEmail);
            var btnEdit = FindViewById<Button>(Resource.Id.btnEdit);

            btnEdit.Click += delegate
            {
                settings.EditUrl(edtPort.Text, edtAddress.Text);
                //int number;
                //if (int.TryParse(edtAddress.Tag.ToString(), out number) == false)
                //{
                //    Console.WriteLine("Write correct value");
                //}
                //else
                //{
                //    History history = new History()
                //    {

                //        Id = int.Parse(edtAddress.Tag.ToString()),
                //        Address = edtAddress.Text,
                //        Port = int.Parse(edtPort.Text),
                //        //Email = edtEmail.Text
                //    };
                //    //db.updateTableHistory(history);
                //    //LoadData();
                //}
            };
        }
    }
}