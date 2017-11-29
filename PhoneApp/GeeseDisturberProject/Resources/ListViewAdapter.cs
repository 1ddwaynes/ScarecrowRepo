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
using GeeseDisturberProject.Model;

namespace GeeseDisturberProject.Resources
{
    public class ViewHolder : Java.Lang.Object
    {
        public TextView txtAddress { get; set; }
        public TextView txtPort { get; set; }
    }
    public class ListViewAdapter:BaseAdapter
    {
        private Activity activity;
        private List<History> lstHistory;
        public ListViewAdapter(Activity activity, List<History> lstHistory)
        {
            this.activity = activity;
            this.lstHistory = lstHistory;
        }

        public override int Count
        {
            get
            {
                return lstHistory.Count;
            }
        }

        public override Java.Lang.Object GetItem(int position)
        {
            return null;
        }

        public override long GetItemId(int position)
        {
            return lstHistory[position].Id;
        }

        public override View GetView(int position, View convertView, ViewGroup parent)
        {
            var view = convertView ?? activity.LayoutInflater.Inflate(Resource.Layout.list_view_dataTemplate, parent, false);

            var txtPort = view.FindViewById<TextView>(Resource.Id.textView1);
            var txtAddress = view.FindViewById<TextView>(Resource.Id.textView2);

            txtAddress.Text = lstHistory[position].Address;
            txtPort.Text = ""+lstHistory[position].Port;

            return view;
        }
    }
}