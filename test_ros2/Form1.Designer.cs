namespace test_ros2
{
    partial class Form1
    {
        /// <summary>
        ///  Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        ///  Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        ///  Required method for Designer support - do not modify
        ///  the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            cylinder_up = new Button();
            cylinder_stop = new Button();
            cylinder_down = new Button();
            labelIPAddress = new Label();
            labelTicks = new Label();
            SuspendLayout();
            // 
            // cylinder_up
            // 
            cylinder_up.Font = new Font("맑은 고딕", 36F, FontStyle.Regular, GraphicsUnit.Point, 129);
            cylinder_up.Location = new Point(744, 24);
            cylinder_up.Name = "cylinder_up";
            cylinder_up.Size = new Size(490, 205);
            cylinder_up.TabIndex = 0;
            cylinder_up.Text = "Cylinder up";
            cylinder_up.UseVisualStyleBackColor = true;
            cylinder_up.Click += button1_Click;
            // 
            // cylinder_stop
            // 
            cylinder_stop.Font = new Font("맑은 고딕", 36F, FontStyle.Regular, GraphicsUnit.Point, 129);
            cylinder_stop.Location = new Point(744, 263);
            cylinder_stop.Name = "cylinder_stop";
            cylinder_stop.Size = new Size(490, 205);
            cylinder_stop.TabIndex = 1;
            cylinder_stop.Text = "Cylinder stop";
            cylinder_stop.UseVisualStyleBackColor = true;
            cylinder_stop.Click += button2_Click;
            // 
            // cylinder_down
            // 
            cylinder_down.Font = new Font("맑은 고딕", 36F, FontStyle.Regular, GraphicsUnit.Point, 129);
            cylinder_down.Location = new Point(744, 501);
            cylinder_down.Name = "cylinder_down";
            cylinder_down.Size = new Size(490, 205);
            cylinder_down.TabIndex = 2;
            cylinder_down.Text = "Cylinder down";
            cylinder_down.UseVisualStyleBackColor = true;
            cylinder_down.Click += button3_Click;
            // 
            // labelIPAddress
            // 
            labelIPAddress.AutoSize = true;
            labelIPAddress.Location = new Point(22, 24);
            labelIPAddress.Name = "labelIPAddress";
            labelIPAddress.Size = new Size(131, 25);
            labelIPAddress.TabIndex = 3;
            labelIPAddress.Text = "labelIPAddress";
            // 
            // labelTicks
            // 
            labelTicks.AutoSize = true;
            labelTicks.Font = new Font("맑은 고딕", 26F, FontStyle.Regular, GraphicsUnit.Point, 129);
            labelTicks.Location = new Point(12, 318);
            labelTicks.Name = "labelTicks";
            labelTicks.Size = new Size(257, 70);
            labelTicks.TabIndex = 4;
            labelTicks.Text = "labelTicks";
            // 
            // Form1
            // 
            AutoScaleDimensions = new SizeF(10F, 25F);
            AutoScaleMode = AutoScaleMode.Font;
            ClientSize = new Size(1277, 742);
            Controls.Add(labelTicks);
            Controls.Add(labelIPAddress);
            Controls.Add(cylinder_down);
            Controls.Add(cylinder_stop);
            Controls.Add(cylinder_up);
            Name = "Form1";
            Text = "Form1";
            ResumeLayout(false);
            PerformLayout();
        }

        #endregion

        private Button cylinder_up;
        private Button cylinder_stop;
        private Button cylinder_down;
        private Label labelIPAddress;
        private Label labelTicks;
    }
}
