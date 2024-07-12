using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace test_ros2
{
    public partial class Form2 : Form
    {
        public bool IsManual { get; private set; }
        public Form2()
        {
            InitializeComponent();
        }
        private void manual_Click(object sender, EventArgs e)
        {
            IsManual = true;
            this.DialogResult = DialogResult.OK;
            this.Close();
        }

        private void auto_Click(object sender, EventArgs e)
        {
            IsManual = false;
            this.DialogResult = DialogResult.OK;
            this.Close();
        }
    }
}