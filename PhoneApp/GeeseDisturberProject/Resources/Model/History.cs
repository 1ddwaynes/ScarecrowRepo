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
using SQLite;

namespace GeeseDisturberProject.Model
{
   public class History
    {
        [PrimaryKey, AutoIncrement]

        public int Id { get; set; }

        public int Port { get; set;}

        public string Address { get; set; }

        public bool Use { get; set; }
    }

    public class Setting
    {
        public string Port_n { get; set; }

        public string Address_n { get; set; }

        public bool UseNew { get; set; }

        public string GetUrl(bool settings)
        {
            if (Address_n != null && Port_n != null)
            {
                if (settings == false)
                    return Address_n + Port_n + "/stream";

                else
                    return Address_n + Port_n + "/panel";
            }
            else
            {
                InitUrl();
                return GetUrl(settings);
            }
        }

        public void saveset()
        {
            var prefs = Application.Context.GetSharedPreferences("MyApp", FileCreationMode.Private);
            var prefEditor = prefs.Edit();
            prefEditor.PutString("Address", Address_n);
            prefEditor.PutString("Port", Port_n);

            if (UseNew == true)
            {
                prefEditor.PutString("UseNew", "true");
            }
            else
            {
                prefEditor.PutString("UseNew", "false");
            }
              
            prefEditor.Commit();
        }

        internal void EditUrl(object text1, object text2)
        {
            throw new NotImplementedException();
        }

        public void retrieveset()
        {
            //retreive 
            var prefs = Application.Context.GetSharedPreferences("MyApp", FileCreationMode.Private);
            Address_n = prefs.GetString("Address", null);
            Port_n = prefs.GetString("Port", null);
            var UseNew_n = prefs.GetString("Usenew", null);

            if (UseNew_n == "true")
                UseNew = true;
            else
                UseNew = false;

            //Show a toast
            //RunOnUiThread(() => Toast.MakeText(this, somePref, ToastLength.Long).Show());

        }

        public void EditUrl (string port, string Address)
        {
            this.Port_n = port;

            if (Address == null)
                Address_n = "http://proxy7.remote-iot.com:";
            else
                this.Address_n = Address;

            UseNew = true;

            saveset();
        }

        public void InitUrl()
        {
            if (UseNew != true)
            {
                Address_n = "http://proxy7.remote-iot.com:";

                Port_n = "10274";
            }

        }
    }
}