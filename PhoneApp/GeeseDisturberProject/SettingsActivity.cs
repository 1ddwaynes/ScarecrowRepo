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


            var btnAdd = FindViewById<Button>(Resource.Id.btnAdd);
            var btnEdit = FindViewById<Button>(Resource.Id.btnEdit);
            var btnDelete = FindViewById<Button>(Resource.Id.btnDelete);

            //LoadData
            //LoadData();

            //Event
            btnAdd.Click += delegate
            {
                //History history = new History()
                //{
                //    //Id = int.Parse(edtAddress.Tag.ToString()),
                //    Address = edtAddress.Text,
                //    Port = int.Parse(edtPort.Text),
                //    //Email = edtEmail.Text
                //};
                //db.insertIntoTableHistory(history);
                //LoadData();
            };

            btnEdit.Click += delegate {
                int number;
                if (int.TryParse(edtAddress.Tag.ToString(), out number) == false)
                {
                    Console.WriteLine("Write correct value");
                }
                else
                {
                    History history = new History()
                    {

                        Id = int.Parse(edtAddress.Tag.ToString()),
                        Address = edtAddress.Text,
                        Port = int.Parse(edtPort.Text),
                        //Email = edtEmail.Text
                    };
                    //db.updateTableHistory(history);
                    //LoadData();
                }
            };

            btnDelete.Click += delegate {
                int number;
                if (int.TryParse(edtAddress.Tag.ToString(), out number) == false)
                {
                    Console.WriteLine("Write correct value");
                }
                else
                {
                    History history = new History()
                    {
                        Id = int.Parse(edtAddress.Tag.ToString()),
                        Address = edtAddress.Text,
                        Port = int.Parse(edtPort.Text),
                        //Email = edtEmail.Text
                    };
                    //db.deleteTableHistory(history);
                    //LoadData();
                }
            };

            //lstData.ItemClick += (s, e) => {
            //    //Set background for selected item
            //    for (int i = 0; i < lstData.Count; i++)
            //    {
            //        if (e.Position == i)
            //            lstData.GetChildAt(i).SetBackgroundColor(Android.Graphics.Color.DarkGray);
            //        else
            //            lstData.GetChildAt(i).SetBackgroundColor(Android.Graphics.Color.Transparent);

            //    }

            //    //Binding Data
            //    var txtAddress = e.View.FindViewById<TextView>(Resource.Id.textView2);
            //    var txtPort = e.View.FindViewById<TextView>(Resource.Id.textView1);
            //    //var txtEmail = e.View.FindViewById<TextView>(Resource.Id.textView3);

            //    edtAddress.Text = txtAddress.Text;
            //    edtAddress.Tag = e.Id;

            //    edtPort.Text = txtPort.Text;

            //    //edtEmail.Text = txtEmail.Text;
            //};

        }

        //private void LoadData()
        //{
        //    lstSource = db.selectTableHistory();
        //    var adapter = new ListViewAdapter(this, lstSource);
        //    lstData.Adapter = adapter;
        //}
    }
}