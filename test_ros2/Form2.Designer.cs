namespace test_ros2
{
    partial class Form2
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
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
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            manual = new Button();
            auto = new Button();
            SuspendLayout();
            // 
            // manual
            // 
            manual.Location = new Point(80, 170);
            manual.Name = "manual";
            manual.Size = new Size(261, 127);
            manual.TabIndex = 0;
            manual.Text = "Manual";
            manual.UseVisualStyleBackColor = true;
            manual.Click += manual_Click;
            // 
            // auto
            // 
            auto.Location = new Point(419, 170);
            auto.Name = "auto";
            auto.Size = new Size(261, 127);
            auto.TabIndex = 1;
            auto.Text = "Auto";
            auto.UseVisualStyleBackColor = true;
            auto.Click += auto_Click;
            // 
            // Form2
            // 
            AutoScaleDimensions = new SizeF(10F, 25F);
            AutoScaleMode = AutoScaleMode.Font;
            ClientSize = new Size(800, 450);
            Controls.Add(auto);
            Controls.Add(manual);
            Name = "Form2";
            Text = "Form2";
            ResumeLayout(false);
        }

        #endregion

        private Button manual;
        private Button auto;
    }
}