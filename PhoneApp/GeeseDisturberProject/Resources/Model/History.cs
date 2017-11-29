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

        //public bool Use { get; set; }
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

        public void EditUrl (string port, string Address)
        {
            this.Port_n = port;

            this.Address_n = Address;

            UseNew = true;
        }

        public void InitUrl()
        {
            if (UseNew != true)
            {
                Address_n = "http://proxy7.remote-iot.com:";

                Port_n = "12356";
            }
        }
    }
}