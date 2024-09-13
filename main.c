#include <gtk/gtk.h>

int main(int argc, char *argv[]) {
  gtk_init(&argc, &argv);

  GtkBuilder *builder = gtk_builder_new();
  if (!gtk_builder_add_from_file(builder, "./main.glade", NULL)) {
    g_printerr("Error loading file\n");
    return -1;
  }

  GtkWidget *window =
      GTK_WIDGET(gtk_builder_get_object(builder, "BSplineWidget"));
  gtk_widget_show_all(window);
  gtk_main();

  return 0;
}
