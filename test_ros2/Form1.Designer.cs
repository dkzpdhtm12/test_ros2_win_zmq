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
            emergency_stop = new Button();
            joint1_value = new Label();
            joint2_value = new Label();
            joint4_value = new Label();
            joint3_value = new Label();
            joint5_value = new Label();
            joint6_value = new Label();
            joint1_value_up = new Button();
            joint1_value_down = new Button();
            joint2_value_up = new Button();
            joint3_value_up = new Button();
            joint4_value_up = new Button();
            joint5_value_up = new Button();
            joint6_value_up = new Button();
            joint2_value_down = new Button();
            joint3_value_down = new Button();
            joint4_value_down = new Button();
            joint5_value_down = new Button();
            joint6_value_down = new Button();
            manipulator_connect = new Button();
            connection_status = new Label();
            SuspendLayout();
            // 
            // cylinder_up
            // 
            cylinder_up.Font = new Font("맑은 고딕", 36F, FontStyle.Regular, GraphicsUnit.Point, 129);
            cylinder_up.Location = new Point(1379, 24);
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
            cylinder_stop.Location = new Point(1379, 260);
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
            cylinder_down.Location = new Point(1379, 490);
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
            // emergency_stop
            // 
            emergency_stop.Location = new Point(1379, 730);
            emergency_stop.Name = "emergency_stop";
            emergency_stop.Size = new Size(490, 226);
            emergency_stop.TabIndex = 5;
            emergency_stop.Text = "Emergency stop";
            emergency_stop.UseVisualStyleBackColor = true;
            emergency_stop.Click += emergency_stop_Click;
            // 
            // joint1_value
            // 
            joint1_value.AutoSize = true;
            joint1_value.Location = new Point(370, 1107);
            joint1_value.Name = "joint1_value";
            joint1_value.Size = new Size(60, 25);
            joint1_value.TabIndex = 6;
            joint1_value.Text = "label1";
            // 
            // joint2_value
            // 
            joint2_value.AutoSize = true;
            joint2_value.Location = new Point(539, 1107);
            joint2_value.Name = "joint2_value";
            joint2_value.Size = new Size(60, 25);
            joint2_value.TabIndex = 7;
            joint2_value.Text = "label1";
            // 
            // joint4_value
            // 
            joint4_value.AutoSize = true;
            joint4_value.Location = new Point(884, 1107);
            joint4_value.Name = "joint4_value";
            joint4_value.Size = new Size(60, 25);
            joint4_value.TabIndex = 8;
            joint4_value.Text = "label1";
            // 
            // joint3_value
            // 
            joint3_value.AutoSize = true;
            joint3_value.Location = new Point(715, 1107);
            joint3_value.Name = "joint3_value";
            joint3_value.Size = new Size(60, 25);
            joint3_value.TabIndex = 9;
            joint3_value.Text = "label1";
            // 
            // joint5_value
            // 
            joint5_value.AutoSize = true;
            joint5_value.Location = new Point(1063, 1107);
            joint5_value.Name = "joint5_value";
            joint5_value.Size = new Size(60, 25);
            joint5_value.TabIndex = 10;
            joint5_value.Text = "label1";
            // 
            // joint6_value
            // 
            joint6_value.AutoSize = true;
            joint6_value.Location = new Point(1229, 1107);
            joint6_value.Name = "joint6_value";
            joint6_value.Size = new Size(60, 25);
            joint6_value.TabIndex = 11;
            joint6_value.Text = "label1";
            // 
            // joint1_value_up
            // 
            joint1_value_up.Location = new Point(339, 1002);
            joint1_value_up.Name = "joint1_value_up";
            joint1_value_up.Size = new Size(123, 83);
            joint1_value_up.TabIndex = 12;
            joint1_value_up.Text = "up";
            joint1_value_up.UseVisualStyleBackColor = true;
            // 
            // joint1_value_down
            // 
            joint1_value_down.Location = new Point(339, 1160);
            joint1_value_down.Name = "joint1_value_down";
            joint1_value_down.Size = new Size(123, 83);
            joint1_value_down.TabIndex = 13;
            joint1_value_down.Text = "down";
            joint1_value_down.UseVisualStyleBackColor = true;
            // 
            // joint2_value_up
            // 
            joint2_value_up.Location = new Point(508, 1002);
            joint2_value_up.Name = "joint2_value_up";
            joint2_value_up.Size = new Size(123, 83);
            joint2_value_up.TabIndex = 14;
            joint2_value_up.Text = "up";
            joint2_value_up.UseVisualStyleBackColor = true;
            // 
            // joint3_value_up
            // 
            joint3_value_up.Location = new Point(682, 1002);
            joint3_value_up.Name = "joint3_value_up";
            joint3_value_up.Size = new Size(123, 83);
            joint3_value_up.TabIndex = 15;
            joint3_value_up.Text = "up";
            joint3_value_up.UseVisualStyleBackColor = true;
            // 
            // joint4_value_up
            // 
            joint4_value_up.Location = new Point(855, 1002);
            joint4_value_up.Name = "joint4_value_up";
            joint4_value_up.Size = new Size(123, 83);
            joint4_value_up.TabIndex = 16;
            joint4_value_up.Text = "up";
            joint4_value_up.UseVisualStyleBackColor = true;
            // 
            // joint5_value_up
            // 
            joint5_value_up.Location = new Point(1031, 1002);
            joint5_value_up.Name = "joint5_value_up";
            joint5_value_up.Size = new Size(123, 83);
            joint5_value_up.TabIndex = 17;
            joint5_value_up.Text = "up";
            joint5_value_up.UseVisualStyleBackColor = true;
            // 
            // joint6_value_up
            // 
            joint6_value_up.Location = new Point(1201, 1002);
            joint6_value_up.Name = "joint6_value_up";
            joint6_value_up.Size = new Size(123, 83);
            joint6_value_up.TabIndex = 18;
            joint6_value_up.Text = "up";
            joint6_value_up.UseVisualStyleBackColor = true;
            // 
            // joint2_value_down
            // 
            joint2_value_down.Location = new Point(508, 1160);
            joint2_value_down.Name = "joint2_value_down";
            joint2_value_down.Size = new Size(123, 83);
            joint2_value_down.TabIndex = 19;
            joint2_value_down.Text = "down";
            joint2_value_down.UseVisualStyleBackColor = true;
            // 
            // joint3_value_down
            // 
            joint3_value_down.Location = new Point(682, 1160);
            joint3_value_down.Name = "joint3_value_down";
            joint3_value_down.Size = new Size(123, 83);
            joint3_value_down.TabIndex = 20;
            joint3_value_down.Text = "down";
            joint3_value_down.UseVisualStyleBackColor = true;
            // 
            // joint4_value_down
            // 
            joint4_value_down.Location = new Point(855, 1160);
            joint4_value_down.Name = "joint4_value_down";
            joint4_value_down.Size = new Size(123, 83);
            joint4_value_down.TabIndex = 21;
            joint4_value_down.Text = "down";
            joint4_value_down.UseVisualStyleBackColor = true;
            // 
            // joint5_value_down
            // 
            joint5_value_down.Location = new Point(1031, 1160);
            joint5_value_down.Name = "joint5_value_down";
            joint5_value_down.Size = new Size(123, 83);
            joint5_value_down.TabIndex = 22;
            joint5_value_down.Text = "down";
            joint5_value_down.UseVisualStyleBackColor = true;
            // 
            // joint6_value_down
            // 
            joint6_value_down.Location = new Point(1201, 1160);
            joint6_value_down.Name = "joint6_value_down";
            joint6_value_down.Size = new Size(123, 83);
            joint6_value_down.TabIndex = 23;
            joint6_value_down.Text = "down";
            joint6_value_down.UseVisualStyleBackColor = true;
            // 
            // manipulator_connect
            // 
            manipulator_connect.Location = new Point(63, 1066);
            manipulator_connect.Name = "manipulator_connect";
            manipulator_connect.Size = new Size(206, 106);
            manipulator_connect.TabIndex = 24;
            manipulator_connect.Text = "Manipulator connect";
            manipulator_connect.UseVisualStyleBackColor = true;
            manipulator_connect.Click += manipulator_connect_Click;
            // 
            // connection_status
            // 
            connection_status.AutoSize = true;
            connection_status.Location = new Point(140, 1189);
            connection_status.Name = "connection_status";
            connection_status.Size = new Size(60, 25);
            connection_status.TabIndex = 25;
            connection_status.Text = "label1";
            // 
            // Form1
            // 
            AutoScaleDimensions = new SizeF(10F, 25F);
            AutoScaleMode = AutoScaleMode.Font;
            ClientSize = new Size(1901, 1283);
            Controls.Add(connection_status);
            Controls.Add(manipulator_connect);
            Controls.Add(joint6_value_down);
            Controls.Add(joint5_value_down);
            Controls.Add(joint4_value_down);
            Controls.Add(joint3_value_down);
            Controls.Add(joint2_value_down);
            Controls.Add(joint6_value_up);
            Controls.Add(joint5_value_up);
            Controls.Add(joint4_value_up);
            Controls.Add(joint3_value_up);
            Controls.Add(joint2_value_up);
            Controls.Add(joint1_value_down);
            Controls.Add(joint1_value_up);
            Controls.Add(joint6_value);
            Controls.Add(joint5_value);
            Controls.Add(joint3_value);
            Controls.Add(joint4_value);
            Controls.Add(joint2_value);
            Controls.Add(joint1_value);
            Controls.Add(emergency_stop);
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
        private Button emergency_stop;
        private Label joint1_value;
        private Label joint2_value;
        private Label joint4_value;
        private Label joint3_value;
        private Label joint5_value;
        private Label joint6_value;
        private Button joint1_value_up;
        private Button joint1_value_down;
        private Button joint2_value_up;
        private Button joint3_value_up;
        private Button joint4_value_up;
        private Button joint5_value_up;
        private Button joint6_value_up;
        private Button joint2_value_down;
        private Button joint3_value_down;
        private Button joint4_value_down;
        private Button joint5_value_down;
        private Button joint6_value_down;
        private Button manipulator_connect;
        private Label connection_status;
    }
}
